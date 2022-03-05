package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class MoveRightClimberDown extends CommandBase {
    private boolean done = false;

    @Override
    public void initialize() {
        this.done = false;

        addRequirements(Robot.rightClimber);
    }

    @Override
    public void execute() {
        if (!done) {
            if (!Robot.rightClimber.isBottomRightSwitchPressed()) {
                // While the front switch is not pressed, keep running the climber Extender Motor out.
                Robot.rightClimber.lower();
            } else {
                Robot.rightClimber.stop();
                done = true;
            }
        } else {
            Robot.rightClimber.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.rightClimber.stop();
    }

    @Override
    public boolean isFinished() {
        return this.done;
    }
}
