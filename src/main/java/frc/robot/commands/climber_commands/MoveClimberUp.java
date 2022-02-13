package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;

public class MoveClimberUp extends CommandBase {
    
    @Override
    public void initialize() {
        addRequirements(Robot.climber);
    }

    
    @Override
    public void execute() {
        Robot.climber.moveClimberUp();
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