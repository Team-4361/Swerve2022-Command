package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import me.wobblyyyy.pathfinder2.geometry.Angle;
import me.wobblyyyy.pathfinder2.revrobotics.SparkMaxMotor;

import static frc.robot.Constants.MotorFlip.ADJUSTOR_FLIPPED;
import static frc.robot.Constants.ShooterAdjustor.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class AngleAdjustSubsystem extends SubsystemBase{
    
    private double target = 0;

    private final SparkMaxMotor adjustor;

    private RelativeEncoder encoder;

    private SparkMaxPIDController cont;

    private final DigitalInput adjustorLimit;

    public AngleAdjustSubsystem(){
        adjustor = SparkMaxMotor.brushless(ADJUSTOR_MOTOR_ID, ADJUSTOR_FLIPPED);

        //isInverted does not work
        adjustor.getSpark().setInverted(true);

        cont = adjustor.getSpark().getPIDController();

        cont.setP((double) 1/8.5);
        cont.setOutputRange(-1, 1);

        encoder = adjustor.getSpark().getEncoder();

        adjustorLimit = new DigitalInput(ADJUSTOR_LIMIT_PORT);

        adjustor.getSpark().enableVoltageCompensation(12);

        zero();
    }


    @Override
    public void periodic() {
        
        cont.setReference(target, ControlType.kPosition, 0);
        

        SmartDashboard.putNumber("Adjustor Rotations:", getPosition());
        SmartDashboard.putNumber("Adjustor Angle:", getAngle());
    }

    
    public void setAngle(double target){
        double targetRotations = target/DEGREES_PER_ROTATION;

        if(targetRotations > MAX_ROTATION){
            this.target = MAX_ROTATION;
        } else if(targetRotations < 0){
            this.target = 0;
        }
        else {
            this.target = target/DEGREES_PER_ROTATION;
        }
    }

    public double getAngle() {
        return getPosition()*DEGREES_PER_ROTATION;
    }

    public void zero(){
        encoder.setPosition(0);
        target = 0;
    }

    public double getPosition(){
        return encoder.getPosition();
    }


}
