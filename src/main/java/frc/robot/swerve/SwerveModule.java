package frc.robot.swerve;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import me.wobblyyyy.pathfinder2.revrobotics.SparkMaxMotor;
import me.wobblyyyy.pathfinder2.utils.StringUtils;

import static frc.robot.Constants.Chassis.SWERVE_WHEEL_CIRCUMFERENCE;

/**
 * A {@code SwerveModule} is composed of two motors and two encoders:
 * a drive motor/encoder and a turn motor/encoder. The turn motor is
 * responsible for controlling the direction the drive motor faces, essentially
 * allowing the robot to move in any direction.
 */
public class SwerveModule {
    // private static final int COUNTS_PER_REV = 4_096;

    private final SparkMaxMotor driveMotor;
    private final SparkMaxMotor turnMotor;
    private final RelativeEncoder driveEncoder;
    private final DutyCycleEncoder rotationPWMEncoder;

    private final double offset;
    private final double errorFactor;

    private final PIDController turnController = new PIDController(
            0.5,
            0.0,
            0.0,
            0.02
    );

    public SwerveModule(int driveMotorId,
                        int turnMotorId,
                        int digitalEncoderPort,
                        double offset,
                        double errorFactor) {
        driveMotor = SparkMaxMotor.brushless(driveMotorId);
        turnMotor = SparkMaxMotor.brushless(turnMotorId);

        driveEncoder = driveMotor.getSpark().getEncoder();
        rotationPWMEncoder = new DutyCycleEncoder(digitalEncoderPort);

        this.offset = offset;
        this.errorFactor = errorFactor;
    }

    public double velocityMetersPerSecond() {
        // rpm -> rps -> mps
        double rotationsPerMinute = driveEncoder.getVelocity();
        double rotationsPerSecond = rotationsPerMinute / 60;

        return rotationsPerSecond * SWERVE_WHEEL_CIRCUMFERENCE;
    }

    public Rotation2d getTurnAngle() {
        return new Rotation2d(turnAngleRadians());
    }

    public double turnAngleRadians() {
        return offset + (rotationPWMEncoder.get() * 2 * Math.PI);
    }

    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getTurnAngle());

        double turnPower = turnController.calculate(
                turnAngleRadians(),
                state.angle.getRadians()
        );

        //driveMotor.setPower(state.speedMetersPerSecond * errorFactor);
        //turnMotor.setPower(turnPower);
    }

    /**
     * get the swerve module's state based on the drive motor's velocity
     * (meters/sec) and the turn encoder's angle.
     *
     * @return a new {@code SwerveModuleState}, representing the module's
     * current state, based on the module's drive motor velocity (m/s)
     * and the turn encoder's angle.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                velocityMetersPerSecond(),
                getTurnAngle()
        );
    }

    public void updateDashboard(String prefix) {
        String driveVelocity = StringUtils.format("%s: vel", prefix);
        String drivePower = StringUtils.format("%s: pow", prefix);
        String turnPower = StringUtils.format("%s: turn pow", prefix);
        String turnPosition = StringUtils.format("%s: turn pos", prefix);

        SmartDashboard.putNumber(driveVelocity, velocityMetersPerSecond());
        SmartDashboard.putNumber(turnPower, turnMotor.getPower());
        SmartDashboard.putNumber(turnPosition, getTurnAngle().getRadians());
        SmartDashboard.putNumber(drivePower, driveMotor.getPower());
    }

    /**
     * get the elapsed distance, in rotations.
     *
     * @return the elapsed distance, in rotations
     */
    public double getDistance() {
        return driveEncoder.getPosition();
    }
}
