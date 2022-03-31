package frc.robot.commands.shooter_commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutoAdjustShooterAngle extends CommandBase{

    private Map<String, Double> targetData;

    private double targetAngle = 0;

    public AutoAdjustShooterAngle(){
        addRequirements(Robot.adjustor); 
    }

    @Override
    public void execute() {
        targetData = Robot.shooterCamera.getTargetGoal();

        if(targetData.get("Status") != 0){
            targetAngle = getRequiredTargetAngle(targetData.get("Distance"));

            Robot.adjustor.setAngle(targetAngle);
        }


    }

    //equation that converts distance to hub to required shooter angle
    public double getRequiredTargetAngle(double distanceToHub){
        double requiredTargetAngle = 0;

        requiredTargetAngle = (2.21*Math.pow(Math.E, 0.504*distanceToHub))*1.15;

        if(requiredTargetAngle > 30){
            requiredTargetAngle = 30;
        } else if(requiredTargetAngle < 0) {
            requiredTargetAngle = 0;
        }
        
        return requiredTargetAngle;
    }

    @Override
    public boolean isFinished() {
        return targetAngle != 0;
    }


    
}
