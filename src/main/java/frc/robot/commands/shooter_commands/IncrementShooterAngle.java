package frc.robot.commands.shooter_commands;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.ShooterAdjustor.*;

public class IncrementShooterAngle extends CommandBase {
    private double targetAngle = 0;

    public void resetAngle() {
        targetAngle = 0;
        Robot.adjustor.setAngle(0);
    }

    @Override
    public void initialize() {
        if (Robot.adjustor.getAngle()+5 < ADJUSTOR_ANGLE_MAX) {
            targetAngle += 5;
            Robot.adjustor.setAngle(targetAngle);
        } else {
            targetAngle = 0;
            Robot.adjustor.setAngle(targetAngle);
        }
    }

    public double getCurrentTargetAngle() {
        return targetAngle;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
