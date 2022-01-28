package frc.robot.subsystems;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StorageSubsystem
 extends SubsystemBase {

    private CANSparkMax loaderMotor, acceptorMotor;
    private DigitalInput photoElectricSensorOne, photoElectricSensorTwo;
    private ColorSensorV3 colorSensor;

    public enum AcceptColor {
        RED, BLUE
    }

    public StorageSubsystem(int loaderMotorPort, int acceptorPort, int photoElectricPortOne, int photoElectricPortTwo, Port colorSensorPort) {
        this.loaderMotor = new CANSparkMax(laoderMotorPort, kBrushless);
        this.acceptorMotor = new CANSparkMax(acceptorPort, kBrushless);
        this.colorSensor = new ColorSensorV3(colorSensorPort);
        this.photoElectricSensorOne = new DigitalInput(photoElectricPortOne);
				this.photoElectricSensorTwo = new DigitalInput(photoElectricPortTwo);
    }

    /** @return Color Value  */
    public Color getColorValue() { return colorSensor.getColor(); }

    /** @return Intake Motor Instance */
    public CANSparkMax getLoaderMotor() { return this.loaderMotor; }

    /** @return Acceptor Motor Instance */
    public CANSparkMax getAcceptorMotor() { return this.acceptorMotor; }
    
    /** @return Color Sensor Instance */
    public ColorSensorV3 getColorSensor() { return this.colorSensor; }

		private ArrayList<StorageTask> storageTasks = new ArrayList<>();

    //Sense the ball, get its color then decide to bring it into the first or second position
		@Override
  	public void periodic() {
    	if(photoElectricSensorOne.get()){
					StorageTask currentStorageTask = new StorageTask(colorSensor, colorSensor);
					storageTasks.add(currentStorageTask);
			}

			if(!photoElectricSensorTwo.get() && storageTasks.size() > 0){
				if(storageTasks.get(0).isColorFound()){
					
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
}


class StorageTask{

	private boolean colorFound = false;

	private boolean taskFaulty = false;

	private ColorSensorV3 colorSensor;

	private long taskCreatedTime;

	private StorageSubsystem.AcceptColor;

	private Task acceptOrDeny =  null;
	
	private Thread findColorThread;


	public StorageTask(ColorSensorV3 colorSensor, StorageSubsystem.AcceptColor targetBallColor){
		this.colorSensor = colorSensor;
		this.taskCreatedTime = System.currentTimeMillis();
		this.targetBallColor = targetBallColor;
		this.findColorthread = new Thread(findColor);

		this.findColorThread.start();

	}

	private Runnable findColor = () -> {
		private long elaspsedTime; System.currentTimeMillis() - taskCreatedTime;

		while(!colorFound){
			if(targetBallColor == StorageSubsystem.AcceptColor.BLUE){
				if(colorSensor.getColor().blue > 0.35 && colorSensor.getProximity() > 2000) { 
					acceptOrDeny = Task.ACCEPT;
					colorFound = true;
				}
				else if(colorSensor.getColor().red > 0.35 && colorSensor.getProximity() > 2000) {
					acceptOrDeny = Task.DENY
					colorFound = true;
				}
			} 
			else {
					if(targetBallColor == StorageSubsystem.AcceptColor.BLUE){
						if(colorSensor.getColor().red > 0.35 && colorSensor.getProximity() > 2000) { 
						acceptOrDeny = Task.ACCEPT;
						colorFound = true;
					} else if(colorSensor.getColor().blue > 0.35 && colorSensor.getProximity() > 2000) {
							acceptOrDeny = Task.DENY;
							colorFound = true;
					}
				}
			}

			if(elapsedTime > 1000){
				taskFaulty = true;
				findColorThread.stop();
			}
		}
	}


	public enum Task {
      ACCEPT, DENY
  }

	public boolean isColorFound(){
		return colorFound;
	}

	public boolean isTaskFaulty(){
		return taskFaulty;
	}

	public Task getTask(){
		return acceptOrDeny;
	}

}