package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import me.wobblyyyy.pathfinder2.geometry.PointXYZ;
import me.wobblyyyy.pathfinder2.robot.AbstractOdometry;
import me.wobblyyyy.pathfinder2.time.Time;
import me.wobblyyyy.pathfinder2.utils.StringUtils;
import me.wobblyyyy.pathfinder2.wpilib.WPIAdapter;

import java.util.function.Supplier;

/**
 * the chassis' swerve drive odometry system. this uses encoders on each
 * of the swerve modules, as well as a gyroscope on the robot, to determine
 * the robot's position by integrating its velocity over time. to increase
 * accuracy, this will only update every 5 milliseconds. if it updates too
 * frequently, outlier velocity readings will impact the robot's position and
 * it'll be wrong. if it doesn't update frequently enough, the angle of each
 * of the wheels won't be accounted for properly, which will also make
 * the robot's position wrong
 */
public class SwerveOdometry extends AbstractOdometry {
    private final double updateInterval =
            Constants.Chassis.ODOMETRY_MS_INTERVAL;

    private final SwerveChassis chassis;
    private final Supplier<Rotation2d> gyroSupplier;
    private final SwerveDriveOdometry odometry;
    private Pose2d pose;
    private double lastUpdateTimeMs;

    public SwerveOdometry(SwerveChassis chassis,
                          Supplier<Rotation2d> gyroSupplier,
                          Pose2d pose) {
        this.chassis = chassis;
        this.gyroSupplier = gyroSupplier;
        this.pose = pose;
        odometry = new SwerveDriveOdometry(
                chassis.getSwerveKinematics(),
                gyroSupplier.get(),
                pose
        );
    }

    private static String formatState(SwerveModuleState state) {
        return StringUtils.format(
                "v: %s a: %s deg",
                state.speedMetersPerSecond,
                state.angle.getDegrees()
        );
    }

    public void update() {
        // each of these states is m per sec and omega rad per sec
        SwerveModuleState frontRightState = chassis.getFrontRight().getState();
        SwerveModuleState frontLeftState = chassis.getFrontLeft().getState();
        SwerveModuleState backRightState = chassis.getBackRight().getState();
        SwerveModuleState backLeftState = chassis.getBackLeft().getState();

        SmartDashboard.putString("FR State", formatState(frontRightState));
        SmartDashboard.putString("FL State", formatState(frontLeftState));
        SmartDashboard.putString("BR State", formatState(backRightState));
        SmartDashboard.putString("BL State", formatState(backLeftState));

        pose = odometry.update(
                gyroSupplier.get(),
                frontRightState,
                frontLeftState,
                backRightState,
                backLeftState
        );

        lastUpdateTimeMs = Time.ms();
    }

    public void reset() {
        odometry.resetPosition(new Pose2d(), gyroSupplier.get());
        odometry.resetPosition(new Pose2d(), gyroSupplier.get());
    }

    public Pose2d getPose() {
        return pose;
    }

    public boolean shouldUpdate() {
        // only update the odometry every X milliseconds
        // updating it too frequently may cause very inaccurate results
        return Time.ms() - updateInterval >= lastUpdateTimeMs;
    }


    @Override
    public PointXYZ getRawPosition() {
        //if (shouldUpdate())
        //    update();

        //return WPIAdapter.pointXYZFromPose(pose);
        return new PointXYZ();
    }
}
