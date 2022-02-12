package frc.robot.subsystems;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.IntakeShooter.SHOOTER_MOTOR_PORT;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot_utils.MotorUtil;



public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax shooterMotor = new CANSparkMax(SHOOTER_MOTOR_PORT, kBrushless);
    private CANSparkMax acceptorMotor, storageMotor;
    private RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
    private PIDController shooterController = new PIDController(0, 1, 0);
    private StorageSubsystem storageSubsystem;

    public ShooterSubsystem(StorageSubsystem subsystem) {
        this.storageSubsystem = subsystem;

        // Pull the motors from the Storage Subsystem
        this.acceptorMotor = storageSubsystem.getAcceptorMotor();
        this.storageMotor = storageSubsystem.getStorageMotor();
    }

    @Override
    public void periodic() {}

    public double getVelocity(){
        return shooterEncoder.getVelocity();
    }

    public void setShooterMotor(double val){
        MotorUtil.runMotor(shooterMotor, val);
    }

    public int getBallsLoaded() {
        return storageSubsystem.getBallsLoaded();
    }

    public void setStorageMotor(double val) {
        MotorUtil.runMotor(storageMotor, val);
    }

    public boolean acceptorSensorCovered() {
        return storageSubsystem.getAcceptorSensorCovered();
    }

    public boolean storageSensorCovered() {
        return storageSubsystem.getStorageSensorCovered();
    }

    /*
    public void setShooterWheelVelocity(double speed){
        MotorUtil.runMotor(shooterMotor, MathUtil.clamp(shooterController.calculate(getVelocity(), speed), -1.0, 1.0));
    }
    */

    public void runShooterTimed(double speed, int timeMs) {
        MotorUtil.runMotorTimed(shooterMotor, speed, timeMs);
    }

    public void stopShooter() {
        shooterMotor.set(0);
    }

    public double getShooterCurrent() {
        return shooterMotor.getOutputCurrent();
    }


    public boolean isDesiredSpeed(double speed){
        return (shooterEncoder.getVelocity() > speed) ? true : false;
    }

    public void resetPID(){
        shooterController.reset();
    }
}
