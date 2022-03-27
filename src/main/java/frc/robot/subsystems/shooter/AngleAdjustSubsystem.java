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

public class AngleAdjustSubsystem extends SubsystemBase{
    
    private double target = 0;

    private final SparkMaxMotor adjustor;

    private RelativeEncoder encoder;

    private PIDController controller = new PIDController((double) 1 / 10.5, 0, 0);

    private double power = 0;

    private double delta = 0;

    private final DigitalInput adjustorLimit;

    public AngleAdjustSubsystem(){
        adjustor = SparkMaxMotor.brushless(ADJUSTOR_MOTOR_ID, ADJUSTOR_FLIPPED);

        adjustor.setIsInverted(true);

        encoder = adjustor.getSpark().getEncoder();

        adjustorLimit = new DigitalInput(ADJUSTOR_LIMIT_PORT);

        adjustor.getSpark().enableVoltageCompensation(12);
    }



    @Override
    public void periodic() {

        if(adjustorLimit.get()){
            adjustor.getSpark().stopMotor();
        } else {
            delta = target - getPosition();

            // TODO Auto-generated method stub
            power = controller.calculate(delta);
    
            adjustor.setPower(power);
        }

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
            this.target = target;
        }
    }

    public double getAngle() {
        return encoder.getPosition()*DEGREES_PER_ROTATION;
    }

    public void zero(){
        encoder.setPosition(0);
        target = 0;
    }

    public double getPosition(){
        return encoder.getPosition();
    }


}
