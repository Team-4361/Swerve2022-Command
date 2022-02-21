package frc.robot.subsystems;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.IntakeShooter.*;
import static frc.robot.Constants.MotorValue.*;
import static frc.robot.Constants.MotorFlip.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.robot_utils.MotorUtil;


public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooterMotor, acceptorMotor, storageMotor, adjustMotor;
    private final RelativeEncoder shooterEncoder, adjustEncoder;
    private final PIDController shooterController = new PIDController(0, 1, 0);
    private final StorageSubsystem storageSubsystem;

    public ShooterSubsystem(StorageSubsystem subsystem) {
        this.storageSubsystem = subsystem;

        // Pull the motors from the Storage Subsystem
        this.acceptorMotor = storageSubsystem.getAcceptorMotor();
        this.storageMotor = storageSubsystem.getStorageMotor();

        this.shooterMotor = new CANSparkMax(SHOOTER_MOTOR_PORT, kBrushless);
        this.adjustMotor = new CANSparkMax(SHOOTER_ADJUSTMENT_PORT, kBrushless);

        this.shooterEncoder = shooterMotor.getEncoder();
        this.adjustEncoder = adjustMotor.getEncoder();
    }

    @Override public void periodic() {}

    public double getVelocity() { return shooterEncoder.getVelocity(); }
    public void setShooterMotor(double val) { MotorUtil.runMotor(shooterMotor, val); }
    public int getBallsLoaded() { return storageSubsystem.getBallsLoaded(); }
    public void setStorageMotor(double val) { MotorUtil.runMotor(storageMotor, val); }
    public boolean acceptorSensorCovered() { return storageSubsystem.getAcceptorSensorCovered(); }
    public boolean storageSensorCovered() { return storageSubsystem.getStorageSensorCovered(); }

    public void setShooterVelocity(double speed) {
        MotorUtil.runMotor(shooterMotor, MathUtil.clamp(shooterController.calculate(getVelocity(), speed), -1.0, 1.0));
    }

    public void runShooterTimed(double speed, int timeMs) { MotorUtil.runMotorTimed(shooterMotor, speed, timeMs); }
    public void stopShooter() { shooterMotor.set(0); }
    public double getShooterCurrent() { return shooterMotor.getOutputCurrent(); }

    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    public boolean isDesiredSpeed(double speed) { return shooterEncoder.getVelocity() > speed; }

    public void resetPID() { shooterController.reset(); }
}