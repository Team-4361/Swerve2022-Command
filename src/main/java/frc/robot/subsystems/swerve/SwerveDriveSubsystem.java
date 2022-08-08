package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.SwerveChassis;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.OptionalDouble;

/**
 * This {@link SwerveDriveSubsystem} is designed to be used for controlling the {@link SwerveChassis}, and utilizing
 * an {@link AHRS} gyroscope to provide the field-relating driving a robot needs. This is also useful for debugging
 * purposes (or for simple autonomous) as it allows driving in a specific direction.
 */
public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveChassis swerveChassis;
    public final AHRS gyro;

    /** Initializes a new {@link SwerveDriveSubsystem}, and resets the Gyroscope. */
    public SwerveDriveSubsystem() {
        swerveChassis = new SwerveChassis();
        gyro = new AHRS(SPI.Port.kMXP);
        gyro.reset();
    }
    
    /** @return A {@link HashMap} containing {@link SwerveModuleState} of the robot. */
    public HashMap<String, SwerveModuleState> getSwerveModuleStates() {
        return swerveChassis.getSwerveModuleStates();
    }

    /**
     * Drives the robot in a specific direction, using a field-relative control. Held <b>indefinitely</b> until
     * this method is recalled again.
     * 
     * @param speeds The {@link ChassisSpeeds} to drive the robot with.
     * @see ChassisSpeeds#fromFieldRelativeSpeeds(double, double, double, Rotation2d) 
     */
    public void drive(ChassisSpeeds speeds) {
        swerveChassis.drive(speeds);
    }

    /** Drives the robot to the right direction at 0.8 m/s (possibly?) */
    public void driveRight() {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.8, 0, 0, Rotation2d.fromDegrees(0)));
    }

    /** Drives the robot to the left direction at 0.8 m/s (possibly?) */
    public void driveLeft() {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(-0.8, 0, 0, Rotation2d.fromDegrees(0)));
    }

    /** Drives the robot forward at 0.8 m/s (possibly?) */
    public void driveForward() {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0.8, 0, Rotation2d.fromDegrees(0)));
    }

    /** Drives the robot backwards at 0.8 m/s (possibly?) */
    public void driveBack() {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, -0.8, 0, Rotation2d.fromDegrees(0)));
    }

    /** Stops the robot from moving completely, will usually not release brake mode from testing. */
    public void stop() {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)));
    }

    /** @return {@link Rotation2d} gyroscope instance. */
    public Rotation2d getGyro() {
        return gyro.getRotation2d();
    }

    /**
     * Resets the gyroscope, <b>very important</b> you call this method ONLY at the beginning of the robot turning on,
     * otherwise from testing it will cause issues.
     */
    public void resetGyro() {
        gyro.reset();
    }

    /** @return The distance of the {@link SwerveChassis} */
    public double getDistance() {
        return swerveChassis.getDistance();
    }

    /** @return The currently used {@link SwerveChassis} */
    public SwerveChassis getSwerveChassis(){
        return swerveChassis;
    }
}
