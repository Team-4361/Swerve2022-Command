package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class LeftClimberUpCommand extends CommandBase {
    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        addRequirements(Robot.leftClimber);
    }

    /** The main body of a command. Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        Robot.leftClimber.raise();
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally, or when it
     * interrupted/canceled.
     *
     * <p>Do not schedule commands here that share requirements with this command.
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
