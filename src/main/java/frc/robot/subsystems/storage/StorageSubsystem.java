package frc.robot.subsystems.storage;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import me.wobblyyyy.pathfinder2.robot.components.AbstractMotor;

import static frc.robot.Constants.MotorFlip.ACCEPTOR_FLIPPED;
import static frc.robot.Constants.MotorFlip.STORAGE_FLIPPED;
import static frc.robot.Constants.Storage.*;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class StorageSubsystem extends SubsystemBase {
    private final DigitalInput frontProximity, rearProximity;
    private final ColorSensorV3 indexColorSensor;
    private final AbstractMotor acceptorMotor, storageMotor;

    private AcceptColor acceptColor;

    private StorageTask nextTask;
    private RetractMode retractMode;

    public StorageSubsystem(AcceptColor acceptColor) {
        this.acceptColor = acceptColor;

        CANSparkMax acceptorSpark = new CANSparkMax(ACCEPTOR_MOTOR_PORT, kBrushless);
        CANSparkMax storageSpark = new CANSparkMax(STORAGE_MOTOR_PORT, kBrushless);

        this.frontProximity = new DigitalInput(ACCEPTOR_PHOTO_ELECTRIC_PORT);
        this.rearProximity = new DigitalInput(STORAGE_PHOTO_ELECTRIC_PORT);
        this.indexColorSensor = new ColorSensorV3(COLOR_SENSOR_PORT);

        this.acceptorMotor = new AbstractMotor(
                acceptorSpark::set,
                acceptorSpark::get,
                ACCEPTOR_FLIPPED
        );

        this.storageMotor = new AbstractMotor(
                storageSpark::set,
                storageSpark::get,
                STORAGE_FLIPPED
        );
    }

    @Override
    public void periodic() {
        updateSensors();
    }

    public void updateSensors() {
        int colorProximity = indexColorSensor.getProximity();
        Color color = indexColorSensor.getColor();

        SmartDashboard.putNumber("Storage: Red", color.red);
        SmartDashboard.putNumber("Storage: Green", color.green);
        SmartDashboard.putNumber("Storage: Blue", color.blue);
        SmartDashboard.putNumber("Storage: Proximity", colorProximity);

        // Everything with the Photo Electric sensor is opposite of what it should be.
        SmartDashboard.putBoolean("Storage: Acceptor Loaded", frontProximityCovered());
        SmartDashboard.putBoolean("Storage: Storage Loaded", rearProximityCovered());

        SmartDashboard.putNumber("Storage: Balls Loaded", getBallsLoaded());
    }

    public void setAcceptColor(AcceptColor color) {
        this.acceptColor = color;
    }

    public void setAcceptorMotor(double speed) {
        acceptorMotor.setPower(speed);
    }

    public void setStorageMotor(double speed) {
        storageMotor.setPower(speed);
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

    public Color getColor() {
        return indexColorSensor.getColor();
    }

    public int getProximity() {
        return indexColorSensor.getProximity();
    }

    /**
     * @return Based off the target color, and what is currently detected, return {@link StorageTask#ACCEPT},
     * {@link StorageTask#REJECT}, or {@link StorageTask#NEUTRAL} if nothing should be done.
     */
    public StorageTask getDetectedTask() {
        if (Robot.storage.getProximity() >= PROXIMITY_THRESHOLD) {
            Color detectedColor = getColor();

            switch (getAcceptColor()) {
                case BLUE:
                    if (detectedColor.blue > BLUE_THRESHOLD) {
                        return StorageTask.ACCEPT;
                    } else if (detectedColor.red > RED_THRESHOLD) {
                        return StorageTask.REJECT;
                    } else {
                        return StorageTask.NEUTRAL;
                    }

                case RED:
                    if (detectedColor.red > RED_THRESHOLD) {
                        return StorageTask.ACCEPT;
                    } else if (detectedColor.blue > BLUE_THRESHOLD) {
                        return StorageTask.REJECT;
                    } else {
                        return StorageTask.NEUTRAL;
                    }

                default:
                    return StorageTask.NEUTRAL;
            }
        } else {
            return StorageTask.NEUTRAL;
        }
    }

    /**
     * Sets the next task that should be executed, used for when the proper value is received. This is
     * designed to be used in another CommandBase.
     *
     * @param task The {@link StorageTask} to be used.
     */
    public void setNextTask(StorageTask task) {
        this.nextTask = task;
    }

    /** @return The next {@link StorageTask} that should be used, and resets the original. */
    public StorageTask getNextTask() {
        StorageTask taskBuffer = this.nextTask;
        this.nextTask = null;

        return taskBuffer;
    }

    /**
     * Sets the retraction mode, used for after a Storage command cycle has ended, on the conditions
     * the intake should be retracted.
     *
     * @param mode The {@link RetractMode} to use for the Intake.
     */
    public StorageSubsystem setRetractMode(RetractMode mode) {
        this.retractMode = mode;
        return this;
    }

    /** @return Retract Mode */
    public RetractMode getRetractMode() {
        return this.retractMode;
    }

    /** @return Returns the {@link AcceptColor}. Either {@link AcceptColor#RED}, or {@link AcceptColor#BLUE} */
    public AcceptColor getAcceptColor() {
        return this.acceptColor;
    }

    /** @return If there are the maximum amount of balls stored (2) */
    public boolean isFull() {
        return getBallsLoaded() == 2;
    }
}
