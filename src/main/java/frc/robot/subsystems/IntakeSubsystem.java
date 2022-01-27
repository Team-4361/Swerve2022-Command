package frc.robot.subsystems;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax intakeMotor, acceptorMotor;
    private DigitalInput photoElectricSensor;
    private ColorSensorV3 colorSensor;
    public enum AcceptColor {
        RED, BLUE
    }

    public IntakeSubsystem(int intakePort, int acceptorPort, int photoElectricPort, Port colorSensorPort) {
        this.intakeMotor = new CANSparkMax(intakePort, kBrushless);
        this.acceptorMotor = new CANSparkMax(acceptorPort, kBrushless);
        this.colorSensor = new ColorSensorV3(colorSensorPort);
        this.photoElectricSensor = new DigitalInput(photoElectricPort);
    }

    /** @return Color Value  */
    public Color getColorValue() { return colorSensor.getColor(); }

    /** @return Intake Motor Instance */
    public CANSparkMax getIntakeMotor() { return this.intakeMotor; }

    /** @return Acceptor Motor Instance */
    public CANSparkMax getAcceptorMotor() { return this.acceptorMotor; }
    
    /** @return Color Sensor Instance */
    public ColorSensorV3 getColorSensor() { return this.colorSensor; }

    
    /** 
     * Starts the Intake Motor running, which should be running when the command
     * has started.
     * 
     * @param speed Speed from -1.0 to 1.0 to run the motor at
     * @return IntakeSubsystem
     */
    public IntakeSubsystem startIntakeMotor(double speed) {
        this.intakeMotor.set(speed);
        return this;
    }

    /** 
     * Stops the Intake Motor
     * @return IntakeSubsystem
     */
    public IntakeSubsystem stopIntakeMotor() {
        this.intakeMotor.set(0);
        return this;
    }

    
    /**
     * Starts the Acceptor Motor running, used to accept the ball after reading 
     * the Color Sensor.
     * 
     * @param speed Speed from -1.0 to 1.0 to run the motor at
     * @return IntakeSubsystem
     */
    public IntakeSubsystem startAcceptorMotor(double speed) {
        this.acceptorMotor.set(speed);
        return this;
    }

    /**
     * Stops the Acceptor Motor
     * @return IntakeSubsystem
     */
    public IntakeSubsystem stopAcceptorMotor() {
        this.acceptorMotor.set(0);
        return this;
    }

    /** @return If the PhotoElectricSensor has a close proximity to an object. */
    public boolean getPhotoElectricProximity() {
        return this.photoElectricSensor.get();
    }
}
