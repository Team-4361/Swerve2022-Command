package frc.robot.subsystems;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeShooter;
import frc.robot.Constants.MotorValue;
import frc.robot.robotutils.MotorUtil;
import frc.robot.subsystems.StorageSubsystem.StorageListener;

public class StorageSubsystem extends SubsystemBase {
	public static CANSparkMax storageMotor, acceptorMotor, 
				        leftIntakeExtend, rightIntakeExtend;

	public static DigitalInput acceptorSensor, storageSensor;
	public static ColorSensorV3 colorSensor;
	public static AcceptColor acceptColor;

	public static final double EXTEND_MOTOR_POSITIION = 0;

	// Can be from 0-2 depending on the amount of balls detected in the Storage.
	protected static int ballsLoaded = 0;

	protected static ArrayList<StorageTask> storageTasks = new ArrayList<>();
	protected static ArrayList<StorageListener> storageListeners;

  	public static enum AcceptColor { RED, BLUE }
	public static enum Task { ACCEPT, DENY }

	private double proximity, acceptorCurrent, storageCurrent, acceptorRPM, storageRPM;
	public static CANSparkMax stalledMotor;
	private RelativeEncoder storageEncoder, acceptorEncoder, leftIntakeEncoder, rightIntakeEncoder;
	private Color color;

    public StorageSubsystem(AcceptColor color) {
        acceptorMotor = new CANSparkMax(IntakeShooter.ACCEPTOR_MOTOR_PORT, kBrushless);
        storageMotor = new CANSparkMax(IntakeShooter.STORAGE_MOTOR_PORT, kBrushless);
        colorSensor = new ColorSensorV3(IntakeShooter.COLOR_SENSOR_PORT);
        acceptorSensor = new DigitalInput(IntakeShooter.ACCEPTOR_PHOTO_ELECTRIC_PORT);
		storageSensor = new DigitalInput(IntakeShooter.STORAGE_PHOTO_ELECTRIC_PORT);
		leftIntakeExtend = new CANSparkMax(IntakeShooter.L_INTAKE_MOTOR_PORT, kBrushless);
		rightIntakeExtend = new CANSparkMax(IntakeShooter.R_INTAKE_MOTOR_PORT, kBrushless);

		storageEncoder = storageMotor.getEncoder();
		acceptorEncoder = acceptorMotor.getEncoder();
		leftIntakeEncoder = leftIntakeExtend.getEncoder();
		rightIntakeEncoder = rightIntakeExtend.getEncoder();

		

		acceptColor = color;
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

	public void extendIntake() {
		// Run the motor until we reach the amount of rotations that are required.
		MotorUtil.runMotor(leftIntakeExtend, MotorValue.ACCEPT_SPEED);
		MotorUtil.runMotor(rightIntakeExtend, MotorValue.ACCEPT_SPEED);

		while (Math.abs(leftIntakeEncoder.getPosition()) < IntakeShooter.EXTEND_MOTOR_POSITION && 
		       Math.abs(rightIntakeEncoder.getPosition()) < IntakeShooter.EXTEND_MOTOR_POSITION) {

			// Do nothing
		}

		MotorUtil.runMotor(leftIntakeExtend, 0);
		MotorUtil.runMotor(rightIntakeExtend, 0);
	}

	public void retractIntake() {
		// Run the motor until we reach the amount of rotations that are required.
		MotorUtil.runMotor(leftIntakeExtend, -MotorValue.ACCEPT_SPEED);
		MotorUtil.runMotor(rightIntakeExtend, -MotorValue.ACCEPT_SPEED);

		while (Math.abs(leftIntakeEncoder.getPosition()) != 0 && 
		       Math.abs(rightIntakeEncoder.getPosition()) != 0) {

			// Do nothing
		}

		MotorUtil.runMotor(leftIntakeExtend, 0);
		MotorUtil.runMotor(rightIntakeExtend, 0);
	}

    /** @return Color Value  */
    public Color getColorValue() { return colorSensor.getColor(); }

    /** @return Storage Motor Instance */
    public CANSparkMax getStorageMotor() { return storageMotor; }

    /** @return Acceptor Motor Instance */
    public CANSparkMax getAcceptorMotor() { return acceptorMotor; }
 
    /** @return Color Sensor Instance */
    public ColorSensorV3 getColorSensor() { return colorSensor; }

	/** @return Acceptor Color */
	public AcceptColor getAcceptColor() { return acceptColor; }

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
		MotorUtil.runMotor(storageMotor, speed);
		return this;
	}

    public StorageSubsystem setAcceptorMotor(double speed) {
		MotorUtil.runMotor(acceptorMotor, speed);
		return this;
	}

	public void setAcceptColor(AcceptColor color){
		this.acceptColor = color;
	}

	public int getBallsLoaded() {
		return ballsLoaded;
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