package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.SwerveChassis;
import frc.robot.swerve.SwerveOdometry;

import java.util.HashMap;

/**
 * This {@link SwerveDriveSubsystem} is designed to be used for controlling the {@link SwerveChassis}, and utilizing
 * an {@link AHRS} gyroscope to provide the field-relating driving a robot needs. This is also useful for debugging
 * purposes (or for simple autonomous) as it allows driving in a specific direction.
 */
public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveChassis swerveChassis;

    public final AHRS gyro;

    private final SwerveOdometry odometry;
    private Rotation2d robotHeading = new Rotation2d(0);

    /** Initializes a new {@link SwerveDriveSubsystem}, and resets the Gyroscope. */
    public SwerveDriveSubsystem() {
        swerveChassis = new SwerveChassis();
        gyro = new AHRS(SPI.Port.kMXP);
        odometry = new SwerveOdometry(swerveChassis, this::getRobotHeading, new Pose2d());

        gyro.reset();
        gyro.calibrate();
    }

    public Rotation2d getRobotHeading() {
        return robotHeading;
    }
    @Override
    public void periodic() {
        // Update the robot speed and other information.
        robotHeading = gyro.getRotation2d();

        if (odometry.shouldUpdate()) {
            odometry.update();
        }

        SmartDashboard.putNumber("Robot MPH", swerveChassis.getDriveMPH());
        SmartDashboard.putNumber("Robot Max MPH", swerveChassis.getMaxDriveMPH());
        SmartDashboard.putString("Robot Actual Heading", robotHeading.toString());
        SmartDashboard.putString("Robot Position", odometry.getPose().toString());
    }

    /** @return A {@link HashMap} containing {@link SwerveModuleState} of the robot. */
    public HashMap<String, SwerveModuleState> getSwerveModuleStates() {
        return swerveChassis.getSwerveModuleStates();
    }

    /**
     * Manually drives the robot in a specific direction, using raw {@link ChassisSpeeds}. This is not recommended
     * because it can override the desired driving mode (robot-relative/field-relative) which autoDrive will
     * compensate for. Held <b>indefinitely</b> until this method is recalled again.
     * 
     * @param speeds The {@link ChassisSpeeds} to drive the robot with.
     * @see ChassisSpeeds#fromFieldRelativeSpeeds(double, double, double, Rotation2d) 
     */
    public void drive(ChassisSpeeds speeds) {
        swerveChassis.drive(speeds);
    }

    /**
     * Drives the Robot using specific speeds, which is converted to field-relative or robot-relative
     * automatically. Unlike {@link #drive(ChassisSpeeds)}, it will compensate for the angle of the Robot.
     *
     * @param vX X-direction m/s (+ right, - left)
     * @param vY Y-direction m/s (+ forward, - reverse)
     * @param omega Yaw rad/s (+ right, - left)
     */
    public void autoDrive(double vX, double vY, double omega) {
        this.drive(ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, omega, robotHeading));
    }

    /** Drives the robot to the right direction at 0.8 m/s (possibly?) */
    public void driveRight() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.8, 0, 0, Rotation2d.fromDegrees(0))); }

    /** Drives the robot to the left direction at 0.8 m/s (possibly?) */
    public void driveLeft() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(-0.8, 0, 0, Rotation2d.fromDegrees(0))); }

    /** Drives the robot forward at 0.8 m/s (possibly?) */
    public void driveForward() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0.8, 0, Rotation2d.fromDegrees(0))); }

    /** Drives the robot backwards at 0.8 m/s (possibly?) */
    public void driveBack() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, -0.8, 0, Rotation2d.fromDegrees(0))); }

    /** Stops the robot from moving completely, will usually not release brake mode from testing. */
    public void stop() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0))); }

    /** @return {@link Rotation2d} gyroscope instance. */
    public Rotation2d getGyro() {
        return robotHeading;
    }

    /**
     * Resets the Odometry, which will offset the gyro angle and cause it to become zero while
     * being referenced, essentially resetting the gyroscope.
     */
    public void resetPosition() {
        odometry.reset();
    }

    /** @return The currently used {@link SwerveChassis} */
    public SwerveChassis getSwerveChassis(){
        return swerveChassis;
    }
}
