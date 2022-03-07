package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class MoveRightClimberDown extends CommandBase {

    @Override
    public void initialize() {
        Robot.rightClimber.setDone(Robot.rightClimber.isBottomRightSwitchPressed());

        addRequirements(Robot.rightClimber);
    }

    @Override
    public void execute() {
        if (Robot.rightClimber.isDangerousTemperature()) {
            Robot.rightClimber.setDone(true);
            SmartDashboard.putBoolean("climber: overheat", true);
        } else if (!Robot.rightClimber.getDone()) {
            if (!Robot.rightClimber.isBottomRightSwitchPressed()) {
                // While the front switch is not pressed, keep running the climber Extender Motor out.
                Robot.rightClimber.lower();
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
    }

    @Override
    public boolean isFinished() {
        return Robot.rightClimber.getDone();
    }
}
