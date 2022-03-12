package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ManualMoveRightClimber extends CommandBase {
    private final boolean lowered;

    public ManualMoveRightClimber(boolean lowered) {
        this.lowered = lowered;
    }

    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        addRequirements(Robot.rightClimber);
    }

    /** The main body of a command. Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        if (lowered) {
            Robot.rightClimber.lower();
        } else {
            Robot.rightClimber.raise();
        }
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally, or when it
     * interrupted/canceled.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        Robot.rightClimber.stop();
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end() method and
     * un-schedule it.
     *
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}