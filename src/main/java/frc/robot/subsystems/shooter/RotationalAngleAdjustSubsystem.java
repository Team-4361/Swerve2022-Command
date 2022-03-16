package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot_utils.encoder.ConcurrentRotationalEncoder;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.MotorFlip.ADJUSTOR_FLIPPED;
import static frc.robot.Constants.ShooterAdjustor.*;
import static frc.robot.robot_utils.motor.MotorUtil.getMotorValue;
import static frc.robot.robot_utils.motor.MotorUtil.inTolerance;

public class RotationalAngleAdjustSubsystem extends SubsystemBase {
    private final CANSparkMax adjustMotor;
    private final ConcurrentRotationalEncoder adjustEncoder;
    private final DigitalInput topLimitSwitch;

    private double targetAngle, maximumAngle;
    private double precisionTolerance = 1;
    private boolean adjustingAngle = false;

    public RotationalAngleAdjustSubsystem() {
        this.adjustMotor = new CANSparkMax(ADJUSTOR_MOTOR_ID, kBrushless);
        this.adjustEncoder = new ConcurrentRotationalEncoder(this.adjustMotor, ADJUSTOR_FLIPPED);
        this.topLimitSwitch = new DigitalInput(ADJUSTOR_LIMIT_PORT);
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
     * up and not attempting to move it any further. This run at a significantly slower speed in order for
     * this to be as accurate as possible.
     *
     * @param tolerance The {@link #precisionTolerance} to set.
     * @return {@link RotationalAngleAdjustSubsystem}
     */
    public RotationalAngleAdjustSubsystem setPrecisionTolerance(double tolerance) {
        this.precisionTolerance = tolerance;
        return this;
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
        } else {
            return ADJUSTOR_SPEED;
        }
    }

    public boolean setTargetAngle(double angle) {
        if (atLimitAngle()) {
            return false;
        }

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

    /** @return If the Angle Adjustor is at the maximum angle, meaning the limit switch has been pressed. */
    public boolean atLimitAngle() {
        return topLimitSwitch.get();
    }

    public double getMaximumAngle() {
        return this.maximumAngle;
    }

    public void setMaximumAngle(double angle) {
        this.maximumAngle = angle;
    }

    public void translateUp() {
        this.adjustMotor.set(-getMotorValue(ADJUSTOR_SPEED, ADJUSTOR_FLIPPED));
    }

    public void translateDown() {
        this.adjustMotor.set(getMotorValue(ADJUSTOR_SPEED, ADJUSTOR_FLIPPED));
    }

    public void stop() {
        this.adjustMotor.stopMotor();
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
            } else if (atLimitAngle()) {
                setMaximumAngle(currentAngle);
                this.adjustingAngle = false;
            }
        } else {
            this.adjustMotor.stopMotor();
        }

        SmartDashboard.putNumber("new shooter angle", getAngle());
    }
}