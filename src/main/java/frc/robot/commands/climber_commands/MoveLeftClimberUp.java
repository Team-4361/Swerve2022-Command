package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class MoveLeftClimberUp extends CommandBase {
    private boolean done = false;

    @Override
    public void initialize() {
        this.done = false;

        addRequirements(Robot.leftClimber);
    }

    @Override
    public void execute() {
        if (!done) {
            if (!Robot.leftClimber.isTopLeftSwitchPressed()) {
                // While the front switch is not pressed, keep running the leftClimber Extender Motor out.
                Robot.leftClimber.raise();
            } else {
                Robot.leftClimber.stop();
                done = true;
            }
        } else {
            Robot.leftClimber.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.leftClimber.stop();
    }

    @Override
    public boolean isFinished() {
        return this.done;
    }
}
