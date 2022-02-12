package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import me.wobblyyyy.pathfinder2.geometry.PointXYZ;
import me.wobblyyyy.pathfinder2.robot.AbstractOdometry;
import me.wobblyyyy.pathfinder2.wpilib.WPIAdapter;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveOdometry extends AbstractOdometry {
    private final SwerveChassis chassis;
    private final Gyro gyro;
    private final SwerveDriveOdometry odometry;
    private Pose2d pose;

    public SwerveOdometry(SwerveChassis chassis,
                          Gyro gyro,
                          Pose2d pose) {
        this.chassis = chassis;
        this.gyro = gyro;
        this.pose = pose;
        odometry = new SwerveDriveOdometry(
                chassis.getSwerveKinematics(),
                gyro.getRotation2d(),
                pose
        );
    }

    public void update() {
        Rotation2d rotation = gyro.getRotation2d();

        // each of these states is m per sec and omega rad per sec
        SwerveModuleState frontRightState = chassis.getFrontRight().getState();
        SwerveModuleState frontLeftState = chassis.getFrontLeft().getState();
        SwerveModuleState backRightState = chassis.getBackRight().getState();
        SwerveModuleState backLeftState = chassis.getBackLeft().getState();

        pose = odometry.update(
                rotation,
                frontRightState,
                frontLeftState,
                backRightState,
                backLeftState
        );
    }

    public Pose2d getPose() {
        return pose;
    }

    @Override
    public PointXYZ getRawPosition() {
        return WPIAdapter.pointXYZFromPose(pose);
    }
}
