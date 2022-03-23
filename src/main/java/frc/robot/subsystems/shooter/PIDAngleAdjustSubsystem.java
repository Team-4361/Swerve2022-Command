package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot_utils.motor.MotorUtil;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.robot_utils.encoder.ConcurrentRotationalEncoder;
import me.wobblyyyy.pathfinder2.geometry.Angle;
import me.wobblyyyy.pathfinder2.revrobotics.SparkMaxMotor;

import static frc.robot.Constants.MotorFlip.ADJUSTOR_FLIPPED;
import static frc.robot.Constants.ShooterAdjustor.*;

import java.util.Map;

public class PIDAngleAdjustSubsystem extends SubsystemBase {
    private final SparkMaxMotor adjustor;
    private final ConcurrentRotationalEncoder absoluteEncoder;
    private final PIDController controller;
    private final DigitalInput adjustorLimit;

    private final double maxPower = 0.09;

    private double targetAngle, maximumAngle = ADJUSTOR_ANGLE_MAX;

    public PIDAngleAdjustSubsystem() {
        adjustor = SparkMaxMotor.brushless(ADJUSTOR_MOTOR_ID, ADJUSTOR_FLIPPED);
        absoluteEncoder = new ConcurrentRotationalEncoder(adjustor.getSpark()).setFlipped(ADJUSTOR_FLIPPED).setRPMTolerance(0.5);

        controller = new PIDController((double) 1 / 63, 0, 0);
        controller.setSetpoint(0.0);

        adjustorLimit = new DigitalInput(ADJUSTOR_LIMIT_PORT);
    }

    /** Resets the {@link ConcurrentRotationalEncoder}, and defaults everything to zero. */
    public void zero() {
        absoluteEncoder.reset();
        targetAngle = 0;
    }

    /** @return The {@link SparkMaxMotor} being used for adjusting. */
    public SparkMaxMotor getAdjustor() {
        return this.adjustor;
    }
    
    public double rotationToAngle(double rotation) {
        return ((Math.abs(rotation) * 360) * (1 / ADJUSTOR_GEAR_RATIO));
    }

    /** @return Current Angle from Base */
    public double getAngle() {
        double rawAngle = rotationToAngle(Math.abs(absoluteEncoder.getAbsoluteRotations()));

        if (adjustor.getPower() == 0 && MotorUtil.inTolerance(0, rawAngle, 2) && targetAngle == 0) {
            return 0;
        } else {
            return rotationToAngle(Math.abs(absoluteEncoder.getAbsoluteRotations()));
        }
    }

    public boolean atDesiredAngle(double desired, double tolerance) {
        return MotorUtil.inTolerance(desired, getAngle(), tolerance);
    }

    public void setAngle(double angle) {
        this.targetAngle = angle;
    }

    public void setRotationsFromBase(double position) {
        setAngle(rotationToAngle(position));
    }

    /** @return If the adjustor limit switch is pressed. */
    public boolean isAdjustorLimitPressed() {
        return this.adjustorLimit.get();
    }

    public double getMaximumAngle() {
        return this.maximumAngle;
    }

    public void setMaximumAngle(double angle) {
        this.maximumAngle = angle;
    }

    @Override
    public void periodic() {
        absoluteEncoder.periodic();

        Angle currentAngle = Angle.fixedDeg(getAngle());
        Angle adjustedTargetAngle = Angle.fixedDeg(targetAngle);

        double delta = Angle.minimumDelta(currentAngle, adjustedTargetAngle);
        double adjustorMotorPower = MathUtil.clamp(controller.calculate(delta), -maxPower, maxPower);

        // Check if the top limit switch is pressed, if so then stop the motor and set the maximum angle.
        if (isAdjustorLimitPressed()) {
            adjustor.setPower(0);

            // Since everything is running inside a loop, if the maximum angle is not already set, then
            // make sure it is adjusted.
            if (!MotorUtil.inTolerance(getMaximumAngle(), getAngle(), 1)) {
                setMaximumAngle(getAngle());
            }
        }

        // If the adjustor motor power is a very small number, the brake mode will not 
        // be properly enabled from how it was tested.
        else if (Math.abs(adjustorMotorPower) < 0.02) {
            adjustor.setPower(0);
        } else {
            adjustor.setPower(adjustorMotorPower);
        }

        SmartDashboard.putNumber("Adjustor: PID Angle", getAngle());
        SmartDashboard.putNumber("Adjustor: PID Target", targetAngle);
        SmartDashboard.putNumber("Adjustor: PID Motor Velocity", absoluteEncoder.getVelocity());

        SmartDashboard.putNumber("Adjustor: PID Motor Power", adjustor.getPower());
        SmartDashboard.putNumber("Adjustor: Maximum Angle", getMaximumAngle());

        SmartDashboard.putBoolean("Adjustor: Limit Pressed", isAdjustorLimitPressed());
    }
}
