package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;

public class MoveClimberDown extends CommandBase {
    
    @Override
    public void initialize() {
        addRequirements(Robot.climber);
    }

    
    @Override
    public void execute() {
        Robot.climber.moveClimberDown();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.climber.stopClimber();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}