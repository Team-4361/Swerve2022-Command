package frc.robot.subsystems.storage;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.robot_utils.MotorUtil;

import java.util.ArrayList;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.MotorFlip.ACCEPTOR_FLIPPED;
import static frc.robot.Constants.MotorFlip.STORAGE_FLIPPED;
import static frc.robot.Constants.MotorValue.ACCEPT_SPEED;
import static frc.robot.Constants.Storage.*;
import static frc.robot.robot_utils.MotorUtil.getMotorValue;
import static frc.robot.robot_utils.MotorUtil.runMotor;

public class StorageSubsystem extends SubsystemBase {

    /**
     * If everything is supposed to be running, saves CPU calculations
     * when there is no need to be used.
     */
    private boolean running = false;

    /** The indexer/acceptor motor inside the Storage device itself. */
    private final CANSparkMax indexerMotor;

    /** The adjustor motor used to push the ball through when accepting */
    private final CANSparkMax storageAdjustorMotor;

    private final DigitalInput frontProximity, rearProximity;
    private final ColorSensorV3 indexColorSensor;
    private final AcceptColor acceptColor;
    private final ArrayList<TaskListener> listeners;

    private Color color;

    private int ballsLoaded = 0;
    private int colorProximity = 0;

    public StorageSubsystem(AcceptColor acceptColor) {
        this.acceptColor = acceptColor;

        this.indexerMotor = new CANSparkMax(ACCEPTOR_MOTOR_PORT, kBrushless);
        this.frontProximity = new DigitalInput(ACCEPTOR_PHOTO_ELECTRIC_PORT);
        this.rearProximity = new DigitalInput(STORAGE_PHOTO_ELECTRIC_PORT);

        this.indexColorSensor = new ColorSensorV3(COLOR_SENSOR_PORT);
        this.listeners = new ArrayList<>();

        this.storageAdjustorMotor = new CANSparkMax(STORAGE_MOTOR_PORT, kBrushless);
    }

    public void acceptTask() {
        listeners.forEach(TaskListener::onAcceptTask);

        // Prevent a feedback loop by stopping the system from running again.
        running = false;
    }

    public void rejectTask() {
        listeners.forEach(TaskListener::onRejectTask);

        // Prevent a feedback loop from stopping the system from running.
        running = false;
    }

    public void addListener(TaskListener listener) {
        listeners.add(listener);
    }

    private void checkBall() {
        // Check if the proximity is greater than the specified threshold.
        if (colorProximity > PROXIMITY_THRESHOLD) {
            switch (acceptColor) {
                case BLUE: {
                    if (color.blue > BLUE_THRESHOLD) {
                        acceptTask();
                    } else if (color.red > RED_THRESHOLD) {
                        rejectTask();
                    }
                    break;
                }
                case RED: {
                    if (color.red > RED_THRESHOLD) {
                        acceptTask();
                    } else if (color.blue > BLUE_THRESHOLD) {
                        rejectTask();
                    }
                    break;
                }
            }
        }
    }

    public void updateSensors() {
        this.color = indexColorSensor.getColor();

        SmartDashboard.putNumber("Storage: Red", color.red);
        SmartDashboard.putNumber("Storage: Green", color.green);
        SmartDashboard.putNumber("Storage: Blue", color.blue);
        SmartDashboard.putNumber("Storage: Proximity", colorProximity);

        // Everything with the Photo Electric sensor is opposite of what it should be.
        SmartDashboard.putBoolean("Storage: Acceptor Loaded", frontProximityCovered());
        SmartDashboard.putBoolean("Storage: Storage Loaded", rearProximityCovered());

        SmartDashboard.putNumber("Storage: Balls Loaded", ballsLoaded);
    }

    public void translateIndexMotor(double val) {
        MotorUtil.runMotor(indexerMotor, getMotorValue(val, ACCEPTOR_FLIPPED));
    }

    public void translateAdjustorMotor(double val) {
        MotorUtil.runMotor(storageAdjustorMotor, getMotorValue(val, STORAGE_FLIPPED));
    }

    @Override
    public void periodic() {
        updateSensors();

        if (rearProximityCovered()) {
            if (frontProximityCovered()) {
                ballsLoaded = 2;
            } else {
                ballsLoaded = 1;
            }
        } else {
            ballsLoaded = 0;
        }

        if (this.running) {
            checkBall();
        }
    }

    /**
     * Sets the running state, to be able to shut off when not needed.
     * @param running If the system should be running, true/false
     */
    public void setRunningState(boolean running) {
        this.running = running;
    }

    /** @return Current Running State */
    public boolean getRunningState() {
        return this.running;
    }

    public void spinStorageAccept() {
        translateAdjustorMotor(ACCEPT_SPEED);
    }

    public void spinStorageReject() {
        translateAdjustorMotor(-ACCEPT_SPEED);
    }

    public void spinIndexAccept(boolean externalIntake) {
        translateIndexMotor(ACCEPT_SPEED);

        if (externalIntake) {
            Robot.intake.spinIntakeAccept();
        }
    }

    public void spinIndexReject(boolean externalIntake) {
        translateIndexMotor(-ACCEPT_SPEED);

        if (externalIntake) {
            Robot.intake.spinIntakeReject();
        }
    }

    public void stopIndexMotor() {
        runMotor(indexerMotor, 0);
    }

    public void stopIntakeMotor() {
        Robot.intake.stopIntakeGroup();
    }

    public boolean frontProximityCovered() {
        return !this.frontProximity.get();
    }

    public boolean rearProximityCovered() {
        return !this.rearProximity.get();
    }

    public int getBallsLoaded() {
        return ballsLoaded;
    }
}