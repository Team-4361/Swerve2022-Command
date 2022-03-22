package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
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

    private final double MAX_ROTATION = 67.5;
    private final double DEGREES_TO_ROTATIONS = 2.25;

    private PIDController controller = new PIDController((double) 1 / 63, 0, 0);

    private double power = 0;

    private double delta = 0;

    public AngleAdjustSubsystem(){
        adjustor = SparkMaxMotor.brushless(ADJUSTOR_MOTOR_ID, ADJUSTOR_FLIPPED);
        encoder = adjustor.getSpark().getEncoder();
        
    }

    @Override
    public void periodic() {

        delta = target - encoder.getPosition();

        // TODO Auto-generated method stub
        power = controller.calculate(delta);

        adjustor.setPower(power);
    }

    
    public void setTarget(double target){
        if(encoder.getPosition() < MAX_ROTATION){
            this.target = target*DEGREES_TO_ROTATIONS;
        }
    }


}
