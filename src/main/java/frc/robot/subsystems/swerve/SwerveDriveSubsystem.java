package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.SwerveChassis;
import frc.robot.swerve.SwerveOdometry;
import me.wobblyyyy.pathfinder2.Pathfinder;
import me.wobblyyyy.pathfinder2.control.Controller;
import me.wobblyyyy.pathfinder2.control.ProportionalController;
import me.wobblyyyy.pathfinder2.robot.Drive;
import me.wobblyyyy.pathfinder2.robot.Odometry;
import me.wobblyyyy.pathfinder2.robot.Robot;

import java.util.HashMap;

import static frc.robot.Constants.TestValue.DRIVE_ENABLED;

public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveChassis swerveChassis;
    public final AHRS gyro;
    private final Controller turnController = new ProportionalController(0.01);
    private final Drive drive;
    private final Odometry odometry;
    private final Robot robot;
    private final Pathfinder pathfinder;

    public SwerveDriveSubsystem() {
        swerveChassis = new SwerveChassis();
        gyro = new AHRS(SPI.Port.kMXP);
        gyro.reset();

        drive = swerveChassis;
        odometry = new SwerveOdometry(swerveChassis, gyro, new Pose2d());
        robot = new Robot(drive, odometry);
        pathfinder = new Pathfinder(robot, turnController);
    }

    public Pathfinder getPathfinder() {
        return pathfinder;
    }

    @Override
    public void periodic() {
    }

    public HashMap<String, SwerveModuleState> getSwerveModuleStates() {
        return swerveChassis.getSwerveModuleStates();
    }

    public void drive(ChassisSpeeds speeds) {
        if (DRIVE_ENABLED) {
            swerveChassis.drive(speeds);
        }
    }

    public void driveRight() {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.8, 0, 0, Rotation2d.fromDegrees(0)));
    }

    public void driveLeft() {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(-0.8, 0, 0, Rotation2d.fromDegrees(0)));
    }

    public void driveForward() {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0.8, 0, Rotation2d.fromDegrees(0)));
    }

    public void driveBack() {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, -0.8, 0, Rotation2d.fromDegrees(0)));
    }

    public void stop() {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)));
    }

    public Rotation2d getGyro() {
        return gyro.getRotation2d();
    }

    public void resetGyro() {
        if (DRIVE_ENABLED) {
            gyro.reset();
        }
    }

    public double getDistance() {
        return swerveChassis.getDistance();
    }
}
