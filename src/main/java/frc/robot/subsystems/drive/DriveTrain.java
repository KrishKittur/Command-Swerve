package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Drive Train class
public class DriveTrain extends SubsystemBase {

    // Global Variables
    public static final double robotWidth = Units.inchesToMeters(20.75);
    public static final double robotLength = Units.inchesToMeters(25.75);
    public static final double turnRadius = Math.sqrt((robotWidth * robotWidth) + (robotLength * robotLength))/2.0;
    private final Translation2d frontLeftPosition = new Translation2d(robotLength/2.0, robotWidth/2.0);
    private final Translation2d frontRightPosition = new Translation2d(robotLength/2.0, -robotWidth/2.0);
    private final Translation2d backLeftPosition = new Translation2d(-robotLength/2.0, robotWidth/2.0);
    private final Translation2d backRightPosition = new Translation2d(-robotLength/2.0, -robotWidth/2.0);
    public final SwerveModule frontLeft = new SwerveModule(7, 8, 0, Units.degreesToRadians(287.790582), false, false);
    public final SwerveModule frontRight = new SwerveModule(1, 2, 1, Units.degreesToRadians(103.122711), true, false);
    public final SwerveModule backLeft = new SwerveModule(5, 6, 2, Units.degreesToRadians(165.787514), false, false);
    public final SwerveModule backRight = new SwerveModule(3, 4, 3, Units.degreesToRadians(57.719948), true, true);
    public final AHRS gyro = new AHRS(SPI.Port.kMXP);
    public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition
    );
    public final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());

    // Method to update odometry
    public void updateOdometry() {
        odometry.update(
            gyro.getRotation2d(), 
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        );
    }

    // In the periodic method of this subsystem update the odometry
    @Override
    public void periodic() {
        updateOdometry();
    }






}
