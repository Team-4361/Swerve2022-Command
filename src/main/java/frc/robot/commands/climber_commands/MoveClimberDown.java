package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class MoveClimberDown extends CommandBase {

    private boolean leftDone = false, rightDone = false;

    @Override
    public void initialize() {
        addRequirements(Robot.climber);

        // This may not be required, but it can eliminate a possible issue.
        leftDone = false;
        rightDone = false;
    }


    @Override
    public void execute() {
        // This runs repeatedly until the command is ended.
        if (!Robot.climber.isBottomLeftSwitchPressed()) {
        // While the front switch is not pressed, keep running the climber Extender Motor out.
        Robot.climber.lowerLeftClimber();
        } else {
            Robot.climber.stopLeftClimber();
            leftDone = true;
        }

        // This runs repeatedly until the command is ended.
        if (!Robot.climber.isBottomRightSwitchPressed()) {
            // While the front switch is not pressed, keep running the climber Extender Motor out.
            Robot.climber.lowerRightClimber();
        } else {
            Robot.climber.stopRightClimber();
            rightDone = true;
        }

        if (leftDone && rightDone) {
            Robot.climber.zero();
            end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.climber.stopClimber();
    }

    @Override
    public boolean isFinished() {
        return leftDone && rightDone;
    }
}