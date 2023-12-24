package frc.robot.commands;




import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** A command that will turn the robot to the specified angle. */
public class DriveToDis extends PIDCommand {
  private DriveSubsystem drive;
  private double setPoint;
  
  public DriveToDis(double target, DriveSubsystem drive) {
    super(
        new PIDController(DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD),
        // Close loop on heading
        drive::getBothEncoders,
        // Set reference to target
        target,
        // Pipe output to turn robot
        output -> drive.arcadeDrive(output, 0),
        // Require the drive
        drive);

    this.drive = drive;
    setPoint = target;
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-3, 3);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kDriveToToleranceMeters, DriveConstants.kDriveRateToleranceMetersPerS);
    getController().reset();
    

  }

  @Override
  public void initialize(){
    drive.resetEncoders();
  }

  
  @Override
  public boolean isFinished() {
    System.out.println("length drivin is "+ drive.getBothEncoders());
    System.out.println("the controller is at set point:"+ getController().atSetpoint());
    if(Math.abs(drive.getBothEncoders()) > Math.abs(setPoint) * 0.95 || getController().atSetpoint() ){
      return true;
    }
    return false;
  }
}

