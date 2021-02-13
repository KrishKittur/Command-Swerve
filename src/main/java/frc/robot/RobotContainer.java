package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.JoystickDriveCommand;
import frc.robot.subsystems.drive.DriveTrain;

// Robot Container
public class RobotContainer {

  // Global variables
  private final XboxController controller = new XboxController(0);
  public final DriveTrain drive = new DriveTrain();

  // Constructor
  public RobotContainer() {
    drive.setDefaultCommand(
      new JoystickDriveCommand(
        drive, 
        () -> controller.getY(Hand.kRight), 
        () -> controller.getX(Hand.kRight), 
        () -> controller.getX(Hand.kLeft)
      )
    );
    configureButtonBindings();
  }
  
  // Method to configure the button bindings
  private void configureButtonBindings() {
    new JoystickButton(controller, Button.kStart.value).whenPressed(
      new SequentialCommandGroup(
        new InstantCommand(() -> {drive.gyro.reset(); drive.gyro.calibrate();}, drive),
        new WaitCommand(0.5)
      )
    );
  }

  // Method to get the autonomus command
  public Command getAutonomousCommand() {
    return null;
  }
}
