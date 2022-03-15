package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot_utils.encoder.ConcurrentRotationalEncoder;
import frc.robot.robot_utils.motor.MotorUtil;
import me.wobblyyyy.pathfinder2.geometry.Angle;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.MotorFlip.ADJUSTOR_FLIPPED;
import static frc.robot.Constants.ShooterAdjustor.*;
import static frc.robot.robot_utils.motor.MotorUtil.getMotorValue;
import static frc.robot.robot_utils.motor.MotorUtil.inTolerance;

public class RotationalAngleAdjustSubsystem extends SubsystemBase {
    private final CANSparkMax adjustMotor;
    private final ConcurrentRotationalEncoder adjustEncoder;
    private double targetAngle;

    private double precisionTolerance = 1;
    private double speedTolerance = 2;
    private boolean adjustingAngle = false;

    public RotationalAngleAdjustSubsystem() {
        this.adjustMotor = new CANSparkMax(ADJUSTOR_MOTOR_ID, kBrushless);
        this.adjustEncoder = new ConcurrentRotationalEncoder(this.adjustMotor, ADJUSTOR_FLIPPED);
    }

    /**
     * Resets the {@link ConcurrentRotationalEncoder}, and zeros it out.
     */
    public void reset() {
        this.adjustEncoder.reset();
    }

    /**
     * @return The {@link CANSparkMax} motor used for controlling the Angle Adjustor.
     */
    public CANSparkMax getAdjustMotor() {
        return this.adjustMotor;
    }

    /**
     * @return The {@link ConcurrentRotationalEncoder} encoder associated with this motor.
     */
    public ConcurrentRotationalEncoder getAdjustEncoder() {
        return this.adjustEncoder;
    }

    /**
     * @return The current angle of the Angle Adjustor.
     */
    public double getAngle() {
        return ((Math.abs(adjustEncoder.getAbsoluteRotations()) * 360) * (1 / ADJUSTOR_GEAR_RATIO));
    }

    /**
     * @return If the angle is currently being adjusted.
     */
    public boolean isAdjustingAngle() {
        return this.adjustingAngle;
    }

    /**
     * Sets the Precision Tolerance, how many degrees off the Adjustor is allowed to be off, before giving
     * up and not attempting to move it any further. This follows after the {@link #speedTolerance} and runs
     * at a significantly slower speed in order for this to be as accurate as possible.
     *
     * @param tolerance The {@link #precisionTolerance} to set.
     * @return {@link RotationalAngleAdjustSubsystem}
     */
    public RotationalAngleAdjustSubsystem setPrecisionTolerance(double tolerance) {
        this.precisionTolerance = tolerance;
        return this;
    }

    /**
     * Sets the Speed Tolerance, until the Adjustor Angle is within the {@link #speedTolerance}, before
     * slowing down the motor and attempting to reach the {@link #precisionTolerance}. This runs at a fast
     * speed to reach the target angle as fast as possible.
     *
     * @param tolerance The {@link #speedTolerance} to set.
     * @return {@link RotationalAngleAdjustSubsystem}
     */
    public RotationalAngleAdjustSubsystem setSpeedTolerance(double tolerance) {
        this.speedTolerance = tolerance;
        return this;
    }

    /**
     * @return The current {@link #speedTolerance} used.
     */
    public double getSpeedTolerance() {
        return this.speedTolerance;
    }

    /**
     * @return The current {@link #precisionTolerance} used.
     */
    public double getPrecisionTolerance() {
        return this.precisionTolerance;
    }

    /**
     * @return If the Adjustor is within the {@link #precisionTolerance} of the desired angle.
     */
    public boolean atDesiredAngle(double desired) {
        return (inTolerance(desired, getAngle(), this.precisionTolerance));
    }

    public double getRequiredSpeed() {
        double currentAngle = getAngle();

        if (inTolerance(targetAngle, currentAngle, this.precisionTolerance)) {
            this.adjustingAngle = false;
            return 0;
        } else if (inTolerance(targetAngle, currentAngle, this.speedTolerance)) {
            return 0.15;
        } else {
            return 0.3;
        }
    }

    public boolean setTargetAngle(double angle) {
        if (angle > ADJUSTOR_ANGLE_MAX || angle < ADJUSTOR_ANGLE_MIN) {
            return false;
        }

        if (inTolerance(angle, getAngle(), this.precisionTolerance)) {
            return false;
        } else {
            this.targetAngle = angle;
            this.adjustingAngle = true;
            return true;
        }
    }

    /**
     * This method is called periodically by the {@link CommandScheduler}. Useful for updating subsystem-specific state
     * that you don't want to offload to a {@link Command}. Teams should try to be consistent within their own codebases
     * about which responsibilities will be handled by Commands, and which will be handled here.
     */
    @Override
    public void periodic() {
        adjustEncoder.periodic();

        if (this.adjustingAngle) {
            double currentAngle = getAngle();

            if (inTolerance(targetAngle, currentAngle, this.precisionTolerance)) {
                this.adjustingAngle = false;
            } else if (currentAngle > targetAngle) {
                // Move the adjustor motor in the down direction.
                this.adjustMotor.set(getMotorValue(getRequiredSpeed(), ADJUSTOR_FLIPPED));
            } else if (currentAngle < targetAngle) {
                this.adjustMotor.set(-getMotorValue(getRequiredSpeed(), ADJUSTOR_FLIPPED));
            } else if (currentAngle == targetAngle) {
                // This is almost completely impossible, but do it anyways.
                this.adjustingAngle = false;
            }
        } else {
            this.adjustMotor.stopMotor();
        }
    }
}