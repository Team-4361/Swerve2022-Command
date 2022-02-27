package frc.robot.subsystems.storage;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Storage.*;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class NewStorageSubsystem extends SubsystemBase{
    private final DigitalInput frontProximity, rearProximity;
    private final ColorSensorV3 indexColorSensor;
    private CANSparkMax indexerMotor, acceptorMotor;

    private final AcceptColor acceptColor;

    private int ballsLoaded = 0;
    private AcceptColor currentColor;

    public NewStorageSubsystem(AcceptColor acceptColor) {
        this.acceptColor = acceptColor;

        this.indexerMotor = new CANSparkMax(ACCEPTOR_MOTOR_PORT, kBrushless);
        this.frontProximity = new DigitalInput(ACCEPTOR_PHOTO_ELECTRIC_PORT);
        this.rearProximity = new DigitalInput(STORAGE_PHOTO_ELECTRIC_PORT);

        this.indexColorSensor = new ColorSensorV3(COLOR_SENSOR_PORT);

        this.indexerMotor = new CANSparkMax(STORAGE_MOTOR_PORT, kBrushless);
        this.acceptorMotor = new CANSparkMax(ACCEPTOR_MOTOR_PORT, kBrushless);
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
    }

    public void updateSensors() {
        int colorProximity  = indexColorSensor.getProximity();
        Color color = indexColorSensor.getColor();

        SmartDashboard.putNumber("Storage: Red", color.red);
        SmartDashboard.putNumber("Storage: Green", color.green);
        SmartDashboard.putNumber("Storage: Blue", color.blue);
        SmartDashboard.putNumber("Storage: Proximity", colorProximity);

        // Everything with the Photo Electric sensor is opposite of what it should be.
        SmartDashboard.putBoolean("Storage: Acceptor Loaded", frontProximityCovered());
        SmartDashboard.putBoolean("Storage: Storage Loaded", rearProximityCovered());

        SmartDashboard.putNumber("Storage: Balls Loaded", ballsLoaded);
    }

    public void setAcceptorMotor(double speed){
        acceptorMotor.set(speed);
    }

    public void setStorageMotor(double speed){
        indexerMotor.set(speed);
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

    public Color getColor(){
        return indexColorSensor.getColor();
    }

    public int getProximity(){
        return indexColorSensor.getProximity();
    }

    public void setCurrentColor(AcceptColor color){
        currentColor = color;
    }

    public AcceptColor getCurrentColor(){
        return currentColor;
    }

    public AcceptColor getAcceptColor(){
        return this.acceptColor;
    }

}
