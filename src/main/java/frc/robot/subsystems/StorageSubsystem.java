package frc.robot.subsystems;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeShooter;
import frc.robot.Constants.MotorValue;
import frc.robot.subsystems.StorageSubsystem.StorageListener;

public class StorageSubsystem extends SubsystemBase {
	private CANSparkMax storageMotor, acceptorMotor;
	private DigitalInput acceptorSensor, storageSensor;
	private ColorSensorV3 colorSensor;
	private AcceptColor acceptColor;

	// Can be from 0-2 depending on the amount of balls detected in the Storage.
	protected static int ballsLoaded = 0;

	// Change if there are any issues from the Current Measuring System.
	private final boolean currentMeasuring = true;

	protected static ArrayList<StorageTask> storageTasks = new ArrayList<>();
	protected static ArrayList<StorageListener> storageListeners;

  	public static enum AcceptColor { RED, BLUE }
	public static enum Task { ACCEPT, DENY }

	private double proximity, acceptorCurrent, storageCurrent, acceptorRPM, storageRPM;
	private CANSparkMax stalledMotor;
	private RelativeEncoder storageEncoder, acceptorEncoder;
	private Color color;

    public StorageSubsystem(AcceptColor color) {
        this.acceptorMotor = new CANSparkMax(IntakeShooter.ACCEPTOR_MOTOR_PORT, kBrushless);
        this.storageMotor = new CANSparkMax(IntakeShooter.STORAGE_MOTOR_PORT, kBrushless);
        this.colorSensor = new ColorSensorV3(IntakeShooter.COLOR_SENSOR_PORT);
        this.acceptorSensor = new DigitalInput(IntakeShooter.ACCEPTOR_PHOTO_ELECTRIC_PORT);
		this.storageSensor = new DigitalInput(IntakeShooter.STORAGE_PHOTO_ELECTRIC_PORT);
		this.acceptColor = color;

		SmartDashboard.putBoolean("Storage: Motor Stalled", false);
		SmartDashboard.putNumber("Storage: Stalled #ID", 0);
    }

	private void updateSensors() {
		this.color = colorSensor.getColor();
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


	private boolean isStalled(CANSparkMax motor) {
		if (currentMeasuring) {
			RelativeEncoder encoder = motor.getEncoder();
			double velocity = encoder.getVelocity();

			if (Math.abs(velocity) < MotorValue.STALL_RPM && motor.getOutputCurrent() > MotorValue.STALL_CURRENT) {
				// The motor has been stalled, return true.
				stalledMotor = motor;

				SmartDashboard.putBoolean("Storage: Motor Stalled", true);
				SmartDashboard.putNumber("Storage: Stalled #ID", stalledMotor.getDeviceId());

				return true;
			} else {
				// The motor is not currently stalled, return false.
				return false;
			}
		} else {
			// Current Measuring is disabled, always return false.
			return false;
		}
	}

	public void runMotor(CANSparkMax motor, double speed) {
		new Thread(new Runnable() {
			@Override
			public void run() {
				try {
					// While the motor is not stopped, continously check if the motor is
					// being stalled by anything, and if so then stop the motor and put it
					// in the detection.
					motor.set(speed);

					// Add a delay before checking for stall current to reduce the possibility of the
                	// extra power and low speed of starting the motor to activate the protection.
                	TimeUnit.MILLISECONDS.sleep(100);
					
					while (motor.get() != 0) {
						if (isStalled(motor)) {
							// The motor is stalled, turn off the motor and cancel everything.
							motor.set(0);
						} else {
							motor.set(speed);
						}
					}
					Thread.currentThread().interrupt();
				} catch (InterruptedException e) {}
			}
		}).start();
	}

    /** @return Color Value  */
    public Color getColorValue() { return colorSensor.getColor(); }

    /** @return Storage Motor Instance */
    public CANSparkMax getStorageMotor() { return this.storageMotor; }

    /** @return Acceptor Motor Instance */
    public CANSparkMax getAcceptorMotor() { return this.acceptorMotor; }
 
    /** @return Color Sensor Instance */
    public ColorSensorV3 getColorSensor() { return this.colorSensor; }

    //Sense the ball, get its color then decide to bring it into the first or second position
	@Override
  	public void periodic() {
    	// Detect how many balls there are in the system, and change the variable.
		if (!storageSensor.get()) {
            // If the storage sensor (rear) is covered, we know there is a ball inside of it.
            ballsLoaded = 1;

            if (!acceptorSensor.get()) {
                // In addition, if the other sensor (front) is covered, there are two balls and can't accept more.
                ballsLoaded = 2;
            }
        } else {
            ballsLoaded = 0;
        }

		updateSensors();

		if (proximity > IntakeShooter.PROXIMITY_THRESHOLD) {
			// A ball is approaching the Acceptor area of the Storage module, check if it should be
			// accepted or denied, and send it to the Listener to be handled by the Command.
			StorageTask storageTask = new StorageTask(colorSensor, acceptColor);
			storageTasks.add(storageTask);
		}
  	}

	public StorageSubsystem addListener(StorageListener listener) {
		storageListeners.add(listener);
		return this;
	}

	public StorageSubsystem setStorageMotor(double speed) {
		runMotor(storageMotor, speed);
		return this;
	}

    public StorageSubsystem setAcceptorMotor(double speed) {
		runMotor(acceptorMotor, speed);
		return this;
	}

	public void setAcceptColor(AcceptColor color){
		this.acceptColor = color;
	}

    /** @return If the PhotoElectricSensor has a close proximity to an object. */
    public boolean getAcceptorSensorCovered() {
		return !this.acceptorSensor.get();
    }

	public boolean getStorageSensorCovered() {
        return !this.storageSensor.get();
    }

	public interface StorageListener {
		void colorFound(Task requiredTask, int ballsLoaded);
		void colorTimeoutError();
	}
}


class StorageTask{

	private boolean colorFound = false;
	private boolean taskFaulty = false;
	private ColorSensorV3 colorSensor;
	private Color colorValue;
	private long taskCreatedTime, elapsedTime;
	private StorageSubsystem.AcceptColor targetBallColor;
	private StorageSubsystem.Task acceptOrDeny =  null;
	private Thread findColorThread;

	private Runnable findColor = () -> {
		taskCreatedTime = System.currentTimeMillis();

		while (true) {
			elapsedTime = System.currentTimeMillis() - taskCreatedTime;
			colorValue = colorSensor.getColor();
			
			// This part is for only finding the color, ignoring the PhotoElectric sensors for now.
			switch (targetBallColor) {
				case BLUE: {
					if (colorSensor.getProximity() > IntakeShooter.PROXIMITY_THRESHOLD) {
						if (colorValue.blue > IntakeShooter.BLUE_THRESHOLD) {
							acceptOrDeny = StorageSubsystem.Task.ACCEPT;
							break;
						}
						if (colorValue.red > IntakeShooter.RED_THRESHOLD) {
							acceptOrDeny = StorageSubsystem.Task.DENY;
							break;
						}
					}
				}
				case RED: {
					if (colorSensor.getProximity() > IntakeShooter.PROXIMITY_THRESHOLD) {
						if (colorValue.blue > IntakeShooter.BLUE_THRESHOLD) {
							acceptOrDeny = StorageSubsystem.Task.DENY;
							break;
						}
						if (colorValue.red > IntakeShooter.RED_THRESHOLD) {
							acceptOrDeny = StorageSubsystem.Task.ACCEPT;
							break;
						}
					}
				}
			}

			if (acceptOrDeny != null) {
				// The value from the Color Sensor has been determined, call the Listener accordingly.
				StorageSubsystem.storageListeners.forEach(li -> li.colorFound(acceptOrDeny, StorageSubsystem.ballsLoaded));

				// Remove the Thread from the ArrayList to prevent possible issues.
				StorageSubsystem.storageTasks.remove(this);
				findColorThread.interrupt();
			}

			if (elapsedTime > 1000) {
				// The task has timed out, call the Timeout Error Listener.
				taskFaulty = true;
				StorageSubsystem.storageListeners.forEach(StorageListener::colorTimeoutError);
				
				// Remove the Thread from the ArrayList to prevent possible issues.
				StorageSubsystem.storageTasks.remove(this);
				findColorThread.interrupt();
			}
		}
	};

	public StorageTask(ColorSensorV3 colorSensor, StorageSubsystem.AcceptColor targetBallColor){
		this.colorSensor = colorSensor;
		this.taskCreatedTime = System.currentTimeMillis();
		this.targetBallColor = targetBallColor;
		this.findColorThread = new Thread(findColor);

		this.findColorThread.start();
	}

	public boolean isColorFound(){ return colorFound; }
	public boolean isTaskFaulty(){ return taskFaulty; }
	public StorageSubsystem.Task getTask(){ return acceptOrDeny; }
}