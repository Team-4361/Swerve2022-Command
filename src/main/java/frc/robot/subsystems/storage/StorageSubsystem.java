package frc.robot.subsystems.storage;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.MotorFlip.ACCEPTOR_FLIPPED;
import static frc.robot.Constants.MotorFlip.STORAGE_FLIPPED;
import static frc.robot.Constants.Storage.*;
import static frc.robot.utils.motor.MotorUtil.flip;

public class StorageSubsystem extends SubsystemBase {
    private final DigitalInput frontProximity, rearProximity;
    private final CANSparkMax acceptorMotor, storageMotor;

    public StorageSubsystem() {
        this.acceptorMotor = new CANSparkMax(ACCEPTOR_MOTOR_PORT, kBrushless);
        this.storageMotor = new CANSparkMax(STORAGE_MOTOR_PORT, kBrushless);

        this.frontProximity = new DigitalInput(ACCEPTOR_PHOTO_ELECTRIC_PORT);
        this.rearProximity = new DigitalInput(STORAGE_PHOTO_ELECTRIC_PORT);
    }

    @Override
    public void periodic() {
        updateSensors();
    }

    public void updateSensors() {
        // Everything with the Photo Electric sensor is opposite of what it should be.
        SmartDashboard.putBoolean("Storage: Acceptor Loaded", frontProximityCovered());
        SmartDashboard.putBoolean("Storage: Storage Loaded", rearProximityCovered());

        SmartDashboard.putNumber("Storage: Balls Loaded", getBallsLoaded());
    }

    public void setAcceptorMotor(double speed) {
        acceptorMotor.set(flip(speed, ACCEPTOR_FLIPPED));
    }

    public void setStorageMotor(double speed) {
        storageMotor.set(flip(speed, STORAGE_FLIPPED));
    }

    public boolean frontProximityCovered() {
        return !this.frontProximity.get();
    }

    public boolean rearProximityCovered() {
        return !this.rearProximity.get();
    }

    public int getBallsLoaded() {
        if (rearProximityCovered() && frontProximityCovered()){
            return 2;
        }else if(rearProximityCovered()){
            return 1;
        }else {
            return 0;
        }
    }

    /** @return If there are the maximum amount of balls stored (2) */
    public boolean isFull() {
        return getBallsLoaded() == 2;
    }
}
