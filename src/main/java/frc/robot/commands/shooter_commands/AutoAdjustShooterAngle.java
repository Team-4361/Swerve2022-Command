package frc.robot.commands.shooter_commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutoAdjustShooterAngle extends CommandBase{

    private Map<String, Double> targetData;

    private double targetAngle = 0;

    @Override
    public void initialize() {
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
    
        return 0;
    }

    @Override
    public boolean isFinished() {
        return targetAngle != 0;
    }


    
}
