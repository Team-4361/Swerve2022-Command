package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class MoveClimberUp extends CommandBase {
    private boolean leftDone = false, rightDone = false;


    @Override
    public void initialize() {
        addRequirements(Robot.climber);

        this.leftDone = false;
        this.rightDone = false;
    }

    @Override
    public void execute() {
        // This runs repeatedly until the command is ended.
        if (!leftDone) {
            if (!Robot.climber.isTopLeftSwitchPressed()) {
                // While the front switch is not pressed, keep running the climber Extender Motor out.
                Robot.climber.raiseLeftClimber();
            } else {
                Robot.climber.stopLeftClimber();
                leftDone = true;
            }
        } else {
            Robot.climber.stopLeftClimber();
        }

        // This runs repeatedly until the command is ended.
        if (!rightDone) {
            if (!Robot.climber.isTopRightSwitchPressed()) {
                // While the front switch is not pressed, keep running the climber Extender Motor out.
                Robot.climber.raiseRightClimber();
            } else {
                Robot.climber.stopRightClimber();
                rightDone = true;
            }
        } else {
            Robot.climber.stopRightClimber();
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