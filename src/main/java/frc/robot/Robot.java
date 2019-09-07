package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.networktables.*;

public class Robot extends TimedRobot { 
  private static DifferentialDrive m_robotDrive;
  private static XboxController pilot;
  private static XboxController copilot;
  private static Solenoid hatchSolenoid; 
  private static Compressor comp;

  private final int buttonB = 2;
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;  

  @Override
  public void robotInit() {
    m_robotDrive = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));

    pilot = new XboxController(0);
    copilot = new XboxController(1);
    hatchSolenoid = new Solenoid(0);
    comp = new Compressor();
    comp.start();
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    Update_Limelight_Tracking();

    double steer =  pilot.getX();
    double drive = -pilot.getY();
    boolean auto = pilot.getAButton();

    steer *= 0.70;
    drive *= 0.70;

    if(auto){
      if(m_LimelightHasValidTarget){
        m_robotDrive.arcadeDrive(m_LimelightDriveCommand, m_LimelightSteerCommand);
      }
      else {
        m_robotDrive.arcadeDrive(0.0, 0.0);
      }
    } else {
      m_robotDrive.arcadeDrive(drive, steer);
    }

    if (copilot.getRawButton(buttonB))
      hatchSolenoid.set(true);
    else 
      hatchSolenoid.set(false);
  }

  @Override
  public void testPeriodic() {
  }

  public void Update_Limelight_Tracking()
  {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.03;                    // how hard to turn toward the target
        final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

        // Getting data from limelight network table
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
  }
}
