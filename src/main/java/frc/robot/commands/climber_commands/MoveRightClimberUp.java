package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class MoveRightClimberUp extends CommandBase {

    @Override
    public void initialize() {
        Robot.rightClimber.setDone(Robot.rightClimber.isTopRightSwitchPressed());

        addRequirements(Robot.rightClimber);
    }

    @Override
    public void execute() {
        if (Robot.rightClimber.isDangerousTemperature()) {
            Robot.rightClimber.setDone(true);
            // SmartDashboard.putBoolean("climber: overheat", true);
        } else if (!Robot.rightClimber.getDone()) {
            if (!Robot.rightClimber.isTopRightSwitchPressed()) {
                // While the front switch is not pressed, keep running the climber Extender Motor out.
                Robot.rightClimber.raise();
            } else {
                Robot.rightClimber.stop();
                Robot.rightClimber.setDone(true);
            }
        } else {
            Robot.rightClimber.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.rightClimber.stop();
        Robot.rightClimber.zero();
    }

    @Override
    public boolean isFinished() {
        return Robot.rightClimber.getDone();
    }
}
