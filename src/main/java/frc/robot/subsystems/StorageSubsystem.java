package frc.robot.subsystems;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.IntakeShooter.*;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeShooter;
import frc.robot.robot_utils.MotorUtil;
import frc.robot.subsystems.StorageSubsystem.AcceptColor;
import frc.robot.subsystems.StorageSubsystem.StorageListener;
import frc.robot.subsystems.StorageSubsystem.Task;

public class StorageSubsystem extends SubsystemBase {
    public static CANSparkMax storageMotor, acceptorMotor;

    public static DigitalInput acceptorSensor, storageSensor;
    public static ColorSensorV3 colorSensor;
    public static AcceptColor acceptColor;

    /**
     * Can be from 0-2 depending on the amount of balls detected in the Storage.
     */
    protected static int ballsLoaded = 0;

    protected static StorageTask currentStorageTask;
    protected static ArrayList<StorageListener> storageListeners;

    public enum AcceptColor {RED, BLUE}

    public enum Task {ACCEPT, DENY}

    private double proximity, acceptorCurrent, storageCurrent, acceptorRPM, storageRPM;

    public static CANSparkMax stalledMotor;
    private final RelativeEncoder storageEncoder;
    private final RelativeEncoder acceptorEncoder;

    /**
     * Creates a new StorageSubsystem with an accepting color, either RED or BLUE.
     *
     * @param color RED or BLUE
     */
    public StorageSubsystem(AcceptColor color) {
        acceptorMotor = new CANSparkMax(IntakeShooter.ACCEPTOR_MOTOR_PORT, kBrushless);
        storageMotor = new CANSparkMax(IntakeShooter.STORAGE_MOTOR_PORT, kBrushless);
        colorSensor = new ColorSensorV3(IntakeShooter.COLOR_SENSOR_PORT);
        acceptorSensor = new DigitalInput(IntakeShooter.ACCEPTOR_PHOTO_ELECTRIC_PORT);
        storageSensor = new DigitalInput(IntakeShooter.STORAGE_PHOTO_ELECTRIC_PORT);

        storageEncoder = storageMotor.getEncoder();
        acceptorEncoder = acceptorMotor.getEncoder();

        acceptColor = color;
    }

    /**
     * Updates the sensors for the Subsystem
     */
    private void updateSensors() {
        Color color = colorSensor.getColor();
        this.proximity = colorSensor.getProximity();
        this.acceptorCurrent = acceptorMotor.getOutputCurrent();
        this.storageCurrent = storageMotor.getOutputCurrent();
        this.acceptorRPM = acceptorEncoder.getVelocity();
        this.storageCurrent = storageEncoder.getVelocity();

        SmartDashboard.putNumber("Storage: Red", color.red);
        SmartDashboard.putNumber("Storage: Green", color.green);
        SmartDashboard.putNumber("Storage: Blue", color.blue);
        SmartDashboard.putNumber("Storage: Proximity", proximity);

        // Everything with the Photo Electric sensor is opposite of what it should be.
        SmartDashboard.putBoolean("Storage: Acceptor Loaded", !acceptorSensor.get());
        SmartDashboard.putBoolean("Storage: Storage Loaded", !storageSensor.get());

        SmartDashboard.putNumber("Storage: Acceptor Amps", acceptorCurrent);
        SmartDashboard.putNumber("Storage: Storage Amps", storageCurrent);

        SmartDashboard.putNumber("Storage: Acceptor RPM", acceptorRPM);
        SmartDashboard.putNumber("Storage: Storage RPM", storageRPM);

        SmartDashboard.putNumber("Storage: Balls Loaded", ballsLoaded);
    }

    /**
     * @return Color Value
     */
    public Color getColorValue() {return colorSensor.getColor();}

    /**
     * @return Storage Motor Instance
     */
    public CANSparkMax getStorageMotor() {return storageMotor;}

    /**
     * @return Acceptor Motor Instance
     */
    public CANSparkMax getAcceptorMotor() {return acceptorMotor;}

    /**
     * @return Color Sensor Instance
     */
    public ColorSensorV3 getColorSensor() {return colorSensor;}

    /**
     * @return Acceptor Color
     */
    public AcceptColor getAcceptColor() {return acceptColor;}

    /**
     * Sense the ball, get its color then decide to bring it into the first or second position.
     */
    @Override
    public void periodic() {
        // Detect how many balls there are in the system, and change the variable.
        if (!storageSensor.get()) {
            // If the storage sensor (rear) is covered, we know there is a ball inside it.
            ballsLoaded = 1;

            if (!acceptorSensor.get()) {
                // In addition, if the other sensor (front) is covered, there are two balls and can't accept more.
                ballsLoaded = 2;
            }
        } else {
            ballsLoaded = 0;
        }

        updateSensors();

        if (proximity > PROXIMITY_THRESHOLD) {
            // A ball is approaching the Acceptor area of the Storage module, check if it should be
            // accepted or denied, and send it to the Listener to be handled by the Command.
            if (currentStorageTask != null) {

                // If we use the ArrayList, then it would keep making infinite StorageTasks as long as
                // the proximity sensor is activated. During prolonged holding of the ball, or if something
                // gets stuck (like during Shooter Testing), this caused a Memory Error. By doing this,
                // it makes sure that only 1 storage task can be made at a time.
                currentStorageTask = new StorageTask(colorSensor, acceptColor, STORAGE_DEFAULT_TIMEOUT);
            }
        }
    }

    /**
     * Adds a Storage Listener to check when these events are called.
     *
     * @param listener StorageListener
     */
    public void addListener(StorageListener listener) { storageListeners.add(listener); }

    /**
     * Sets the Storage Motor Speed
     *
     * @param speed Speed from -1.0 to +1.0
     */
    public void setStorageMotor(double speed) { MotorUtil.runMotor(storageMotor, speed); }

    /**
     * Sets the Acceptance Motor Speed
     *
     * @param speed Speed from -1.0 to +1.0
     */
    public void setAcceptorMotor(double speed) { MotorUtil.runMotor(acceptorMotor, speed); }

    /**
     * Sets the Acceptance Color, RED or BLUE
     *
     * @param color Accept Color
     */
    public void setAcceptColor(AcceptColor color) { acceptColor = color; }

    /**
     * @return Balls Loaded Amount
     */
    public int getBallsLoaded() {
        return ballsLoaded;
    }

    /**
     * @return If the Acceptor (front) Sensor has proximity to an object.
     */
    public boolean getAcceptorSensorCovered() {return !acceptorSensor.get();}

    /**
     * @return If the Storage (rear) Sensor has proximity to an object.
     */
    public boolean getStorageSensorCovered() {return !storageSensor.get();}

    public interface StorageListener {
        /**
         * Called when a Color has been found, and the task that should be done
         * based on the information.
         *
         * @param requiredTask Task to Use, Accept/Reject
         * @param ballsLoaded  Balls Loaded Currently
         */
        void colorFound(Task requiredTask, int ballsLoaded);

        /**
         * When the specified timeout has error
         */
        void colorTimeoutError();
    }
}


class StorageTask {
    private Task acceptOrDeny = null;
    private Thread findColorThread;
    private long taskCreatedTime;

    @SuppressWarnings("InfiniteLoopStatement")
    public StorageTask(ColorSensorV3 colorSensor, AcceptColor targetBallColor, long timeout) {
        this.taskCreatedTime = System.currentTimeMillis();

        // This part is for only finding the color, ignoring the PhotoElectric sensors for now.
        Runnable findColor = () -> {
            taskCreatedTime = System.currentTimeMillis();

            while (true) {
                long elapsedTime = System.currentTimeMillis() - taskCreatedTime;
                Color colorValue = colorSensor.getColor();

                // This part is for only finding the color, ignoring the PhotoElectric sensors for now.
                if (colorSensor.getProximity() > PROXIMITY_THRESHOLD) {
                    switch (targetBallColor) {
                        case BLUE: {
                            if (colorValue.blue > BLUE_THRESHOLD) {
                                acceptOrDeny = Task.ACCEPT;
                            } else if (colorValue.red > RED_THRESHOLD) {
                                acceptOrDeny = Task.DENY;
                            }
                            break;
                        }
                        case RED: {
                            if (colorValue.blue > BLUE_THRESHOLD) {
                                acceptOrDeny = Task.DENY;
                            } else if (colorValue.red > RED_THRESHOLD) {
                                acceptOrDeny = Task.ACCEPT;
                            }
                            break;
                        }
                    }
                }

                if (acceptOrDeny != null) {
                    // The value from the Color Sensor has been determined, call the Listener accordingly.
                    StorageSubsystem.storageListeners.forEach(li -> li.colorFound(acceptOrDeny, StorageSubsystem.ballsLoaded));

                    // Make sure to interrupt all the Threads to prevent memory leak.
                    findColorThread.interrupt();
                }

                if (elapsedTime > timeout) {
                    // The task has timed out, call the Timeout Error Listener.
                    StorageSubsystem.storageListeners.forEach(StorageListener::colorTimeoutError);

                    // Make sure to interrupt all the Threads to prevent memory leak.
                    findColorThread.interrupt();
                }
            }
        };

        this.findColorThread = new Thread(findColor);
        this.findColorThread.start();
    }

    /**
     * @return Storage Task
     */
    public Task getTask() {return acceptOrDeny;}
}