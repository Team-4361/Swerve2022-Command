package frc.robot.commands.shooter_commands;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class IncrementShooterAngle extends CommandBase  {
    
    private double currentAngle = 0;

     @Override
     public void initialize() {
        if(currentAngle <= 30){
            currentAngle += 5;
            new SetShooterAngleCommand(currentAngle).schedule();;
        } else {
            currentAngle = 0;
            new SetShooterAngleCommand(0).schedule();;
        }


        Map<String, Double> map = Robot.shooterCamera.getTargetGoal();

        SmartDashboard.putNumber("Current Shooter Angle", currentAngle);
        SmartDashboard.putNumber("Target distance ", map.get("Distance"));
        SmartDashboard.putNumber("Target Pitch ", map.get("Pitch"));
     }

     

    @Override
    public boolean isFinished() {
        return true;
    }
}
