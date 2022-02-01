package frc.robot.subsystems;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.StorageSubsystem.StorageListener;

import static frc.robot.subsystems.StorageSubsystem.AcceptColor;

public class StorageSubsystem
 extends SubsystemBase {

    private CANSparkMax loaderMotor, acceptorMotor;
    private DigitalInput photoElectricSensorOne, photoElectricSensorTwo;
    private ColorSensorV3 colorSensor;
	private AcceptColor acceptColor;

	public final static double RED_THRESHOLD = 0.35, 
					            BLUE_THRESHOLD = 0.35,
								PROXIMITY_THRESHOLD = 2000;

	protected static ArrayList<StorageTask> storageTasks = new ArrayList<>();
	protected static ArrayList<StorageListener> storageListeners;

    public static enum AcceptColor { RED, BLUE }
	public static enum Task { ACCEPT, DENY }

    public StorageSubsystem(int loaderMotorPort, int acceptorPort, int photoElectricPortOne, int photoElectricPortTwo, Port colorSensorPort, AcceptColor color) {
        this.loaderMotor = new CANSparkMax(loaderMotorPort, kBrushless);
        this.acceptorMotor = new CANSparkMax(acceptorPort, kBrushless);
        this.colorSensor = new ColorSensorV3(colorSensorPort);
        this.photoElectricSensorOne = new DigitalInput(photoElectricPortOne);
		this.photoElectricSensorTwo = new DigitalInput(photoElectricPortTwo);
		this.acceptColor = color;
    }

    /** @return Color Value  */
    public Color getColorValue() { return colorSensor.getColor(); }

    /** @return Intake Motor Instance */
    public CANSparkMax getLoaderMotor() { return this.loaderMotor; }

    /** @return Acceptor Motor Instance */
    public CANSparkMax getAcceptorMotor() { return this.acceptorMotor; }
 
    /** @return Color Sensor Instance */
    public ColorSensorV3 getColorSensor() { return this.colorSensor; }

    //Sense the ball, get its color then decide to bring it into the first or second position
	@Override
  	public void periodic() {
    	if (photoElectricSensorOne.get()) {
			StorageTask currentStorageTask = new StorageTask(colorSensor, acceptColor);
			storageTasks.add(currentStorageTask);
		}

		if (!photoElectricSensorTwo.get() && storageTasks.size() > 0) {
			if (storageTasks.get(0).isColorFound()) {
				StorageTask currentStorageTask = new StorageTask(colorSensor, acceptColor);
				storageTasks.add(currentStorageTask);
			}
		}
  	}

    /** 
     * Starts the Intake Motor running, which should be running when the command
     * has started.
     * 
     * @param speed Speed from -1.0 to 1.0 to run the motor at
     * @return StorageSubsystem

     */
    public StorageSubsystem
		 startIntakeMotor(double speed) {
        this.loaderMotor.set(speed);
        return this;
    }

	public StorageSubsystem addListener(StorageListener listener) {
		storageListeners.add(listener);
		return this;
	}

    /** 
     * Stops the Intake Motor
     * @return StorageSubsystem

     */
    public StorageSubsystem stopIntakeMotor() {
        this.loaderMotor.set(0);
        return this;
    }

    
    /**
     * Starts the Acceptor Motor running, used to accept the ball after reading 
     * the Color Sensor.
     * 
     * @param speed Speed from -1.0 to 1.0 to run the motor at
     * @return StorageSubsystem

     */
    public StorageSubsystem
		 startAcceptorMotor(double speed) {
        this.acceptorMotor.set(speed);
        return this;
    }

    /**
     * Stops the Acceptor Motor
     * @return StorageSubsystem

     */
    public StorageSubsystem
		 stopAcceptorMotor() {
        this.acceptorMotor.set(0);
        return this;
    }

    /** @return If the PhotoElectricSensor has a close proximity to an object. */
    public boolean getPhotoElectricOne() {
        return this.photoElectricSensorOne.get();
    }
		public boolean getPhotoElectricTwo() {
        return this.photoElectricSensorTwo.get();
    }

	public interface StorageListener {
		void colorFound(Task requiredTask);
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
			
			switch (targetBallColor) {
				case BLUE: {
					if (colorSensor.getProximity() > StorageSubsystem.PROXIMITY_THRESHOLD) {
						if (colorValue.blue > StorageSubsystem.BLUE_THRESHOLD) {
							acceptOrDeny = StorageSubsystem.Task.ACCEPT;
							break;
						}
						if (colorValue.red > StorageSubsystem.RED_THRESHOLD) {
							acceptOrDeny = StorageSubsystem.Task.DENY;
							break;
						}
					}
				}
				case RED: {
					if (colorSensor.getProximity() > StorageSubsystem.PROXIMITY_THRESHOLD) {
						if (colorValue.blue > StorageSubsystem.BLUE_THRESHOLD) {
							acceptOrDeny = StorageSubsystem.Task.DENY;
							break;
						}
						if (colorValue.red > StorageSubsystem.RED_THRESHOLD) {
							acceptOrDeny = StorageSubsystem.Task.ACCEPT;
							break;
						}
					}
				}
			}

			if (acceptOrDeny != null) {
				// The value from the Color Sensor has been determined, call the Listener accordingly.
				StorageSubsystem.storageListeners.forEach(li -> li.colorFound(acceptOrDeny));

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