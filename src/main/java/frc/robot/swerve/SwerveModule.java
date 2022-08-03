package frc.robot.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Chassis.SWERVE_WHEEL_CIRCUMFERENCE;

/**
 * A {@code SwerveModule} is composed of two motors and two encoders:
 * a drive motor/encoder and a turn motor/encoder. The turn motor is
 * responsible for controlling the direction the drive motor faces, essentially
 * allowing the robot to move in any direction.
 */
public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
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

    /**
     * Creates a new {@link SwerveModule} instance, using the specified parameters.
     * @param driveMotorId The Motor ID used for driving the wheel.
     * @param turnMotorId The Motor ID used for turning the wheel.
     * @param digitalEncoderPort The {@link DigitalInput} ID used for the Encoder.
     * @param offset The offset to use for driving the wheel.
     * @param errorFactor The maximum error factor that is acceptable.
     */
    public SwerveModule(int driveMotorId, int turnMotorId, int digitalEncoderPort, double offset, double errorFactor) {
        driveMotor = new CANSparkMax(driveMotorId, kBrushless);
        turnMotor = new CANSparkMax(turnMotorId, kBrushless);

        driveEncoder = driveMotor.getEncoder();
        rotationPWMEncoder = new DutyCycleEncoder(digitalEncoderPort);

        this.offset = offset;
        this.errorFactor = errorFactor;
    }

    /** @return The current meters per second of the robot. */
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

        driveMotor.set(state.speedMetersPerSecond * errorFactor);
        turnMotor.set(turnPower);
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
        String driveVelocity = prefix + ": m/s";
        String drivePower = prefix + ": pow";
        String turnPower = prefix + ": turn pow";
        String turnPosition = prefix + ": turn rad";


        SmartDashboard.putNumber(driveVelocity, velocityMetersPerSecond());
        SmartDashboard.putNumber(turnPower, turnMotor.get());
        SmartDashboard.putNumber(turnPosition, turnAngleRadians());
        SmartDashboard.putNumber(drivePower, driveMotor.get());
        SmartDashboard.putNumber(prefix + " pwm encoder:", rotationPWMEncoder.get());
    }

    /**
     * get the elapsed distance, in rotations.
     *
     * @return the elapsed distance, in rotations
     */
    public double getDistance() {
        return driveEncoder.getPosition();
    }

    public void resetDriveEncoder(){
        driveEncoder.setPosition(0);
    }
}
