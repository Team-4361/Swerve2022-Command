package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.chassis.ArcadeDriveCommand;
import frc.robot.commands.chassis.SwerveDriveMode;
import frc.robot.swerve.SwerveChassis;
import frc.robot.swerve.SwerveOdometry;
import me.wobblyyyy.pathfinder2.robot.Odometry;

import java.util.HashMap;

/**
 * This {@link SwerveDriveSubsystem} is designed to be used for controlling the {@link SwerveChassis}, and utilizing
 * an {@link AHRS} gyroscope to provide the field-relating driving a robot needs. This is also useful for debugging
 * purposes (or for simple autonomous) as it allows driving in a specific direction.
 */
public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveChassis swerveChassis;
    private final SwerveOdometry swerveOdometry;
    public final AHRS gyro;

    private Rotation2d robotHeading = new Rotation2d(0);
    private Rotation2d driveHeading = new Rotation2d(0);

    private SwerveDriveMode driveMode = SwerveDriveMode.FIELD_RELATIVE;

    /** Initializes a new {@link SwerveDriveSubsystem}, and resets the Gyroscope. */
    public SwerveDriveSubsystem() {
        swerveChassis = new SwerveChassis();
        gyro = new AHRS(SPI.Port.kMXP);
        gyro.reset();
        gyro.calibrate();

        swerveOdometry = new SwerveOdometry(swerveChassis, () -> (robotHeading), new Pose2d());
    }

    public Rotation2d getRobotHeading() {
        return this.robotHeading;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPose();
    }

    @Override
    public void periodic() {
        // Update the robot speed and other information.
        robotHeading = gyro.getRotation2d();

        SmartDashboard.putString("Robot Position", swerveOdometry.toString());
        SmartDashboard.putNumber("Robot MPH", swerveChassis.getDriveMPH());
        SmartDashboard.putNumber("Robot Max MPH", swerveChassis.getMaxDriveMPH());
        SmartDashboard.putString("Robot Actual Heading", robotHeading.toString());
        SmartDashboard.putString("Robot Drive Heading", driveHeading.toString());

        switch (driveMode) {
            case FIELD_RELATIVE:
                SmartDashboard.putString("Robot Drive Mode", "FIELD_RELATIVE");

                // Synchronizes the driving heading with the robot heading since we are using field-relative control.
                driveHeading = robotHeading;
                break;
            case ROBOT_RELATIVE:
                SmartDashboard.putString("Robot Drive Mode", "ROBOT_RELATIVE");
                break;
        }
    }

    /**
     * Sets the current {@link SwerveDriveMode} being used by the Robot. If {@link SwerveDriveMode#FIELD_RELATIVE} is
     * selected, the {@link AHRS} gyroscope will be updated, allowing updated field-relative control. Otherwise, the
     * gyro angle will be frozen, locking the head direction, and it will act robot-relative again.
     * <p></p>
     * This <b>can</b> be called while the Robot is in operation.
     * @param driveMode The {@link SwerveDriveMode} to apply.
     * @return The updated {@link ArcadeDriveCommand} instance.
     */
    public SwerveDriveSubsystem setDriveMode(SwerveDriveMode driveMode) {
        this.driveMode = driveMode;
        return this;
    }

    public SwerveDriveMode getDriveMode() {
        return this.driveMode;
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
        this.drive(ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, omega, driveHeading));
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
     * Resets the gyroscope, <b>very important</b> you call this method ONLY at the beginning of the robot turning on,
     * otherwise from testing it will cause issues.
     */
    public void resetGyro() {
        gyro.reset();
    }

    /** @return The currently used {@link SwerveChassis} */
    public SwerveChassis getSwerveChassis(){
        return swerveChassis;
    }
}
