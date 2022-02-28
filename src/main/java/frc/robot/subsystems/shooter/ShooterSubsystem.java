package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.robot_utils.MotorUtil;
import me.wobblyyyy.pathfinder2.robot.components.AbstractMotor;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.MotorFlip.SHOOTER_FLIPPED;
import static frc.robot.Constants.Shooter.SHOOTER_MOTOR_ID;
import static frc.robot.robot_utils.MotorUtil.getMotorValue;


public class ShooterSubsystem extends SubsystemBase {
    private final AbstractMotor shooterMotor;
    private final RelativeEncoder shooterEncoder;
    private final PIDController shooterController = new PIDController(0.1, 1, 0);

    public ShooterSubsystem() {
        CANSparkMax shooterSpark = new CANSparkMax(SHOOTER_MOTOR_ID, kBrushless);
        this.shooterMotor = new AbstractMotor(
                shooterSpark::set,
                shooterSpark::get,
                SHOOTER_FLIPPED
        );

        this.shooterEncoder = shooterSpark.getEncoder();
    }

    public double getVelocity() {
        return shooterEncoder.getVelocity();
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
    }

    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    public boolean isDesiredSpeed(double speed) {
        return shooterEncoder.getVelocity() > speed;
    }

    public void resetPID() {
        shooterController.reset();
    }
}
