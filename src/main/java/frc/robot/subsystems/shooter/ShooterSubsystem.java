package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import java.util.function.Supplier;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.MotorFlip.SHOOTER_FLIPPED;
import static frc.robot.Constants.Power.SHOOTER_FLYWHEEL_FUSE;
import static frc.robot.Constants.Shooter.FEED_FWD;
import static frc.robot.Constants.Shooter.SHOOTER_MOTOR_ID;
import static frc.robot.utils.motor.MotorUtil.flip;

public class ShooterSubsystem extends SubsystemBase {
    private final RelativeEncoder shooterEncoder;
    private final CANSparkMax shooterMotor;

    private final SparkMaxPIDController sController;

    private double targetVelocity, constantVelocity = 0;

    private final Supplier<Boolean> constantVelocitySupplier = () ->
            !Robot.bms.isOverCurrentLimit() && Robot.bms.getVoltage() > 10;

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter: Instant Velocity", getVelocity());
        SmartDashboard.putNumber("Shooter: Instant Current", shooterMotor.getOutputCurrent());

        SmartDashboard.putNumber("Shooter: Amps", Robot.bms.getBreaker(SHOOTER_FLYWHEEL_FUSE).getCurrent());

        SmartDashboard.putBoolean("Shooter: Constant Velocity Possible", constantVelocitySupplier.get());

        // Check if we are running the Constant Velocity mode to keep shooter spinning, if so then process.
        if (RobotState.isTeleop() && constantVelocity > 0) {
            // Check the Battery to ensure we are not having a large inrush current/voltage drop, typically when
            // the robot starts moving hard. If we continue to run the shooter motor while this is happening, then
            // voltage will dip too low and cause issues from previous attempts.
            if (constantVelocitySupplier.get()) {
                setShooterVelocity(constantVelocity);
            }
        }
    }

    public ShooterSubsystem() {
        shooterMotor = new CANSparkMax(SHOOTER_MOTOR_ID, kBrushless);
        shooterEncoder = shooterMotor.getEncoder();

        sController = shooterMotor.getPIDController();
        shooterMotor.enableVoltageCompensation(12);
        shooterMotor.setInverted(SHOOTER_FLIPPED);

        sController.setP(0);
        sController.setFF(FEED_FWD);

        // adjustable shoot speed
        SmartDashboard.putNumber("Shooter: Shoot RPM", 4500);
    }

    /**
     * Sets the Constant Velocity to run the Shooter motor at during the entire time the robot is running,
     * 0 to disable.
     *
     * @param velocity The RPM to run the Shooter Motor at <b>continuously.</b>
     * @return {@link ShooterSubsystem}
     */
    public ShooterSubsystem setConstantVelocity(int velocity) {
        this.constantVelocity = velocity;
        return this;
    }

    /** @return The current RPM of the Shooter Motor. */
    public double getVelocity() {
        return shooterEncoder.getVelocity();
    }

    /**
     * Sets the Power Level the Shooter Motor will run at.
     * @param val Power Level from 0.0 to 1.0
     */
    public void setShooterPower(double val) {
        shooterMotor.set(flip(val, SHOOTER_FLIPPED));
    }

    /**
     * Sets the RPM the Shooter Motor will run at.
     * @param speed RPM
     */
    public void setShooterVelocity(double speed) {
        sController.setReference(speed, ControlType.kVelocity, 0);
        this.targetVelocity = speed;
    }

    /** @return The current target velocity. */
    public double getTargetVelocity() {
        return this.targetVelocity;
    }

    /** Stops the Shooter Motor from running */
    public void stopShooter() {
        shooterMotor.stopMotor();
    }

    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    public boolean isDesiredSpeed(double speed) {
        return isAcceptableError(speed, getVelocity(), 0.01);
    }

    public boolean isAcceptableError(double speed, double currVelocity, double errorPercentage){

        double lowerBound = speed - (speed* errorPercentage);
        double upperBound = speed + (speed* errorPercentage);

        return currVelocity >= lowerBound && currVelocity <= upperBound;
    }
}
