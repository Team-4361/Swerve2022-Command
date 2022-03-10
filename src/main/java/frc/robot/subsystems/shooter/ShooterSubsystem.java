package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import me.wobblyyyy.pathfinder2.robot.components.AbstractMotor;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.MotorFlip.SHOOTER_FLIPPED;
import static frc.robot.Constants.Shooter.SHOOTER_MOTOR_ID;


public class ShooterSubsystem extends SubsystemBase {
    private final AbstractMotor shooterMotor;
    private final RelativeEncoder shooterEncoder;
    private final CANSparkMax shooterSpark;
    private final PIDController shooterController = new PIDController(0, 2e-4, 0);

    private double lastVelocity, velocityAcc, currentAcc, lastCurrent;

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter: Instant Velocity", getVelocity());
        SmartDashboard.putNumber("Shooter: Instant Current", shooterSpark.getOutputCurrent());
        SmartDashboard.putNumber("Shooter: Instant Voltage", shooterSpark.getBusVoltage());

        SmartDashboard.putNumber("Shooter: Velocity Acc", getVelocityAcceleration());
        SmartDashboard.putNumber("Shooter: Current Acc", getCurrentAcceleration());
    }

    public ShooterSubsystem() {
        shooterSpark = new CANSparkMax(SHOOTER_MOTOR_ID, kBrushless);
        this.shooterMotor = new AbstractMotor(shooterSpark::set, shooterSpark::get, SHOOTER_FLIPPED);

        this.shooterEncoder = shooterSpark.getEncoder();

        this.lastVelocity = getVelocity();
        this.lastCurrent = this.shooterSpark.getOutputCurrent();

        shooterController.setIntegratorRange(-1, 1);
    }

    public double getVelocity() {
        return -shooterEncoder.getVelocity();
    }

    public void setShooterMotor(double val) {
        shooterMotor.setPower(val);
    }

    public int getBallsLoaded() {
        return Robot.storage.getBallsLoaded();
    }

    public void setStorageMotor(double val) {
        Robot.storage.setStorageMotor(val);
    }

    public boolean acceptorSensorCovered() {
        return Robot.storage.frontProximityCovered();
    }

    public boolean storageSensorCovered() {
        return Robot.storage.rearProximityCovered();
    }

    public void setShooterVelocity(double speed) {
        double velocity = MathUtil.clamp(shooterController.calculate(getVelocity(), speed), -1.0, 1.0);
        shooterMotor.setPower(velocity);
    }

    public void stopShooter() {
        shooterMotor.setPower(0);
        shooterController.reset();
    }

    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    public boolean isDesiredSpeed(double speed) {
        return (getVelocity() - 0.05) >= speed;
    }

    public void resetPID() {
        shooterController.reset();
    }

    public void updateAcceleration() {
        // Subtract the current velocity from the lastVelocity, to get the difference.
        this.velocityAcc = getVelocity() - this.lastVelocity;
        this.currentAcc = this.shooterSpark.getOutputCurrent() - this.lastCurrent;

        // Update the lastVelocity to what the velocity is currently, so it can be looped.
        this.lastVelocity = getVelocity();
        this.lastCurrent = this.shooterSpark.getOutputCurrent();
    }

    public double getVelocityAcceleration() {
        return this.velocityAcc;
    }

    public double getCurrentAcceleration() {
        return this.currentAcc;
    }
}
