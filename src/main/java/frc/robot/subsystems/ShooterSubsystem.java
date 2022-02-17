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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Angle", rotationsToAngle(adjustEncoder.getPosition()));
    }

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

    public void setShooterVelocity(double speed){
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


    public boolean isDesiredSpeed(double speed){
        return shooterEncoder.getVelocity() > speed;
    }

    public void resetPID() {
        shooterController.reset();
    }

    ///////////////////////////////////////////////////////////


    private double rotationsToAngle(double rotations) {
        return ((Math.abs(rotations)*360)*(1/ADJUSTOR_GEAR_RATIO));
    }

    private double angleToRotation(double angle) {
        return ((ADJUSTOR_GEAR_RATIO*angle)/360);
    }

    private boolean getRotateDirection(double currentAngle, double newAngle) {
        double oldAngle = Math.abs(currentAngle);
        double setAngle = Math.abs(newAngle);

        if (oldAngle < setAngle) {
            // rotate forward/up
            return true;
        } else {
            // rotate back/down
            return false;
        }
    }

    public void setAdjustRotations(double rotations) {
        // Currently using a 160:1 gear ratio
        if (getRotateDirection(adjustEncoder.getPosition(), rotations)) {
            // Run forwards to move motor up
            MotorUtil.runMotor(adjustMotor, MotorUtil.getMotorValue(ADJUSTOR_SPEED, ADJUSTOR_FLIPPED));
            while (Math.abs(adjustEncoder.getPosition()) <= rotations) {}
        } else {
            // Run backwards to move motor down
            MotorUtil.runMotor(adjustMotor, -MotorUtil.getMotorValue(ADJUSTOR_SPEED, ADJUSTOR_FLIPPED));
            while (Math.abs(adjustEncoder.getPosition()) >= rotations) {}
        }
    
        MotorUtil.stopMotor(adjustMotor);
    }

    public void setAdjustAngle(double angle) {
        setAdjustRotations(angleToRotation(angle));
    }

    public double getAdjustAngle() {
        return rotationsToAngle(Math.abs(adjustEncoder.getPosition()));
    }
}