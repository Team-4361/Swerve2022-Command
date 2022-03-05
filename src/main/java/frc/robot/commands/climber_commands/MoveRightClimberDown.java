package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class MoveRightClimberDown extends CommandBase {
    private boolean done = false;

    @Override
    public void initialize() {
        this.done = false;
    }

    @Override
    public void execute() {
        if (!done) {
            if (!Robot.climber.isBottomRightSwitchPressed()) {
                // While the front switch is not pressed, keep running the climber Extender Motor out.
                Robot.climber.lowerRightClimber();
            } else {
                Robot.climber.stopRightClimber();
                done = true;
            }
        } else {
            Robot.climber.stopRightClimber();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.climber.stopRightClimber();
    }

    @Override
    public boolean isFinished() {
        return this.done;
    }
}
