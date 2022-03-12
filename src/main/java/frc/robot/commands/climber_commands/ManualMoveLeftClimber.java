package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ManualMoveLeftClimber extends CommandBase {
    private final boolean lowered;

    public ManualMoveLeftClimber(boolean lowered) {
        this.lowered = lowered;
    }

    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        addRequirements(Robot.leftClimber);
    }

    /** The main body of a command. Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        if (lowered) {
            Robot.leftClimber.lower();
        } else {
            Robot.leftClimber.raise();
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
        Robot.leftClimber.stop();
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