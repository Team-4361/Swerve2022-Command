package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;

public class MoveIntakeIn extends CommandBase {
    
    @Override
    public void initialize() {
        addRequirements(Robot.intake);
    }

    
    @Override
    public void execute() {
        if(Robot.intake.getPosition() > 0.05) {
            Robot.intake.runIntakeIn();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}