package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveTrain;
import frc.robot.subsystems.drive.SwerveModule.Output;

// The joystick drive command
public class JoystickDriveCommand extends CommandBase {
    
    // Global variables
    private final DriveTrain subsystem;
    private final DoubleSupplier xVelSup;
    private final DoubleSupplier yVelSup;
    private final DoubleSupplier rotSup;

    // Constructor
    public JoystickDriveCommand(DriveTrain subsystem, DoubleSupplier xVelSup, DoubleSupplier yVelSup, DoubleSupplier rotSup) {
        this.subsystem = subsystem;
        this.xVelSup = xVelSup;
        this.yVelSup = yVelSup;
        this.rotSup = rotSup;
        addRequirements(subsystem);
    }

    // In the execute method drive
    @Override
    public void execute() {
        double xVel = -xVelSup.getAsDouble();
        double yVel = -yVelSup.getAsDouble();
        double rot = -rotSup.getAsDouble()/DriveTrain.turnRadius;
        drive(xVel, yVel, rot);
    }

    // Method to drive
    private void drive(double xVel, double yVel, double rot) {
        ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(
            xVel, yVel, rot, subsystem.gyro.getRotation2d()
        );
        SwerveModuleState[] states = subsystem.kinematics.toSwerveModuleStates(speed);
        if (Math.abs(xVel) < 0.01 && Math.abs(yVel) < 0.01 && Math.abs(rot) < 0.01) {
            double crossAngle = new Rotation2d(DriveTrain.robotLength, DriveTrain.robotWidth).getRadians();
            SwerveModuleState fl = new SwerveModuleState(0, new Rotation2d(crossAngle));
            SwerveModuleState fr = new SwerveModuleState(0, new Rotation2d(-crossAngle));
            SwerveModuleState bl = new SwerveModuleState(0, new Rotation2d(-crossAngle));
            SwerveModuleState br = new SwerveModuleState(0, new Rotation2d(crossAngle));
            states[0] = fl;
            states[1] = fr;
            states[2] = bl;
            states[3] = br;
        }
        subsystem.frontLeft.setDesiredState(states[0], Output.PERCENT);
        subsystem.frontRight.setDesiredState(states[1], Output.PERCENT);
        subsystem.backLeft.setDesiredState(states[2], Output.PERCENT);
        subsystem.backRight.setDesiredState(states[3], Output.PERCENT);
    }

}
