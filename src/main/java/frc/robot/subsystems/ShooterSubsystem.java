package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.robot_utils.MotorUtil;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Shooter.SHOOTER_MOTOR_ID;


public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooterMotor, acceptorMotor, storageMotor;
    private final RelativeEncoder shooterEncoder;
    private final PIDController shooterController = new PIDController(0, 1, 0);

    public ShooterSubsystem() {


        // Pull the motors from the Storage Subsystem
        this.acceptorMotor = Robot.storage.getAcceptorMotor();
        this.storageMotor = Robot.storage.getStorageMotor();

        this.shooterMotor = new CANSparkMax(SHOOTER_MOTOR_ID, kBrushless);

        this.shooterEncoder = shooterMotor.getEncoder();
    }

    @Override
    public void periodic() {

    }

    public double getVelocity() {
        return shooterEncoder.getVelocity();
    }

    public void setShooterMotor(double val) {
        MotorUtil.runMotor(shooterMotor, val);
    }

    public int getBallsLoaded() {
        return Robot.storage.getBallsLoaded();
    }

    public void setStorageMotor(double val) {
        MotorUtil.runMotor(storageMotor, val);
    }

    public boolean acceptorSensorCovered() {
        return Robot.storage.getAcceptorSensorCovered();
    }

    public boolean storageSensorCovered() {
        return Robot.storage.getStorageSensorCovered();
    }

    public void setShooterVelocity(double speed) {
        MotorUtil.runMotor(shooterMotor, MathUtil.clamp(shooterController.calculate(getVelocity(), speed), -1.0, 1.0));
    }

    public void runShooterTimed(double speed, int timeMs) {
        MotorUtil.runMotorTimed(shooterMotor, speed, timeMs);
    }

    public void stopShooter() {
        shooterMotor.set(0);
    }

    public double getShooterCurrent() {
        return shooterMotor.getOutputCurrent();
    }

    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    public boolean isDesiredSpeed(double speed) {
        return shooterEncoder.getVelocity() > speed;
    }

    public void resetPID() {
        shooterController.reset();
    }
}