package frc.robot.swerve;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class TestSwerveOdometry {
    private static final double SIZE = 1;

    private double gyroAngle = 0;
    private Gyro gyro = new Gyro() {
        @Override
        public void calibrate() {

        }

        @Override
        public void reset() {

        }

        @Override
        public double getAngle() {
            return gyroAngle;
        }

        @Override
        public double getRate() {
            return 0;
        }

        @Override
        public void close() {

        }
    };
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(SIZE / 2, SIZE / 2),
            new Translation2d(-SIZE / 2, SIZE / 2),
            new Translation2d(SIZE / 2, -SIZE / 2),
            new Translation2d(-SIZE / 2, -SIZE / 2)
    );
    private SwerveDriveOdometry odometry;

    @Before
    public void beforeEach() {
        gyroAngle = 0;

        odometry = new SwerveDriveOdometry(
                kinematics,
                Rotation2d.fromDegrees(gyroAngle)
        );
    }

    @Test
    public void testForwardsOdometry() {
        double speed = 1;
        Rotation2d angle = Rotation2d.fromDegrees(90);

        SwerveModuleState fr = new SwerveModuleState(speed, angle);
        SwerveModuleState fl = new SwerveModuleState(speed, angle);
        SwerveModuleState br = new SwerveModuleState(speed, angle);
        SwerveModuleState bl = new SwerveModuleState(speed, angle);

        odometry.updateWithTime(0, gyro.getRotation2d(), fr, fl, br, bl);
        odometry.updateWithTime(1, gyro.getRotation2d(), fr, fl, br, bl);

        Assert.assertEquals(
                new Pose2d(0, 1, new Rotation2d()),
                odometry.getPoseMeters()
        );
    }

    @Test
    public void testBackwardsOdometry() {
        double speed = -1;
        Rotation2d angle = Rotation2d.fromDegrees(90);

        SwerveModuleState fr = new SwerveModuleState(speed, angle);
        SwerveModuleState fl = new SwerveModuleState(speed, angle);
        SwerveModuleState br = new SwerveModuleState(speed, angle);
        SwerveModuleState bl = new SwerveModuleState(speed, angle);

        odometry.updateWithTime(0, gyro.getRotation2d(), fr, fl, br, bl);
        odometry.updateWithTime(1, gyro.getRotation2d(), fr, fl, br, bl);

        Assert.assertEquals(
                new Pose2d(0, -1, new Rotation2d()),
                odometry.getPoseMeters()
        );
    }
}
