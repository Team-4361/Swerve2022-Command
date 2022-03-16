package frc.robot.commands.shooter_commands;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.ShooterAdjustor.*;

public class IncrementShooterAngle extends CommandBase  {
    
    private double targetAngle = 0, actualAngle;

     @Override
     public void initialize() {
        actualAngle = Robot.adjustor.getAngle();

        if(actualAngle < ADJUSTOR_ANGLE_MAX){
            targetAngle += 5;
            Robot.adjustor.setTargetAngle(targetAngle);
        } else {
            targetAngle = 0;
            Robot.adjustor.setTargetAngle(targetAngle);
        }
     }

    public double getCurrentTargetAngle(){
        return targetAngle;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
