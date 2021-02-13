package frc.robot.subsystems.drive;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpiutil.math.MathUtil;

// Swerve Module Class
public class SwerveModule {

    // Output Enum
    public enum Output {
        PERCENT,
        VELOCITY
    }

    // Global Variables
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final CANEncoder driveEncoder;
    private final AnalogEncoder turnEncoder;
    private final boolean reversed;
    private final boolean encoderInverted;
    PIDController turnFeedback = new PIDController(0.5, 0.0, 0.0001);
    PIDController driveFeedback = new PIDController(0.5, 0.0, 0.0);
    SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.15, 2.9, 0.3);

    // Constructor 
    public SwerveModule(int driveID, int turnID, int turnChannel, double offset, boolean reversed, boolean encoderInverted) {
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = new AnalogEncoder(turnChannel, offset);
        this.reversed = reversed;
        this.encoderInverted = encoderInverted;
        
        driveEncoder.setVelocityConversionFactor((1/60) * (1/7.04) * 2.0 * Math.PI * 0.0508);
    }

    // Method to set the drive by voltage
    public void setDriveVoltage(double voltage) { driveMotor.setVoltage(reversed ? -voltage : voltage); }
    
    // Method to set the drive by duty cycle
    public void setDriveDutyCycle(double dutyCycle) { driveMotor.set(reversed ? -dutyCycle : dutyCycle); }

    // Method to get the drive encoders velocity
    public double getDriveVelocity() { return encoderInverted ? -driveEncoder.getVelocity() : driveEncoder.getVelocity(); }

    // Method to get the current state of the module
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocity(), 
            new Rotation2d(turnEncoder.get())
        );
    }

    // Method to set the desired state of the module
    public void setDesiredState(SwerveModuleState desiredState, Output output) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turnEncoder.get()));
        double turnOutput = MathUtil.clamp(turnFeedback.calculate(turnEncoder.get(), state.angle.getRadians()), -0.5, 0.5);
        
        turnMotor.set(turnOutput);
        if (output == Output.PERCENT) {
            setDriveDutyCycle(state.speedMetersPerSecond);
        } else {
            double driveFb = driveFeedback.calculate(getDriveVelocity(), state.speedMetersPerSecond);
            double driveFf = driveFeedforward.calculate(state.speedMetersPerSecond);
            double driveOutput = MathUtil.clamp(driveFb + driveFf, -12, 12);
            driveMotor.setVoltage(driveOutput);
        }
    }



    
    






    
}
