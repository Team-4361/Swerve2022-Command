package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;

public class RunIntakeOutward extends CommandBase {
    
    @Override
    public void initialize() {
        addRequirements(Robot.intake);
    }

    
    @Override
    public void execute() {
        Robot.intake.runIntakeOut();
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