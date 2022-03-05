package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class MoveLeftClimberDown extends CommandBase {
    private boolean done = false;

    @Override
    public void initialize() {
        this.done = false;
    }

    @Override
    public void execute() {
        if (!done) {
            if (!Robot.climber.isBottomLeftSwitchPressed()) {
                // While the front switch is not pressed, keep running the climber Extender Motor out.
                Robot.climber.lowerLeftClimber();
            } else {
                Robot.climber.stopLeftClimber();
                done = true;
            }
        } else {
            Robot.climber.stopLeftClimber();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.climber.stopLeftClimber();
    }

    @Override
    public boolean isFinished() {
        return this.done;
    }
}
