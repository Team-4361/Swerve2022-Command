package frc.robot.commands.shooter_commands;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.ShooterAdjustor.*;

public class IncrementShooterAngle extends CommandBase {
    private double targetAngle = 0;

    private double incrementLength = 1;

    public void resetAngle() {
        targetAngle = 0;
        incrementLength = 1;
    }

    public IncrementShooterAngle(){
        targetAngle = 0;
    }

    @Override
    public void initialize() {

        if(Robot.adjustor.getAngle()+incrementLength > ADJUSTOR_ANGLE_MAX){
            incrementLength = -1;
        } else if (Robot.adjustor.getAngle()+incrementLength < 0){
            incrementLength = 1;
        }

        targetAngle += incrementLength;
            
        Robot.adjustor.setAngle(targetAngle);


        SmartDashboard.putNumber("Target Angle", targetAngle);
    }

    public double getCurrentTargetAngle() {
        return targetAngle;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
