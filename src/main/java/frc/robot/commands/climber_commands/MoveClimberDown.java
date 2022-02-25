package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;

public class MoveClimberDown extends CommandBase {
    
    @Override
    public void initialize() {
        addRequirements(Robot.climber);
    }

    
    @Override
    public void execute() {
        // This runs repeatedly until the command is ended.
        if (!Robot.climber.isBottomSwitchPressed()) {
            // While the front switch is not pressed, keep running the climber Extender Motor out.
            Robot.climber.lowerClimber();
        } else {
            // The limit switch is pressed, stop the climber and end the command.
            Robot.climber.stopClimber();
            end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.climber.stopClimber();
    }

    @Override
    public boolean isFinished() {
        return Robot.climber.isBottomSwitchPressed();
    }
}