package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class MoveLeftClimberDown extends CommandBase {

    @Override
    public void initialize() {
        Robot.leftClimber.setDone(Robot.leftClimber.isBottomLeftSwitchPressed());

        addRequirements(Robot.leftClimber);
    }

    @Override
    public void execute() {
        if (Robot.leftClimber.isDangerousTemperature()) {
            Robot.leftClimber.setDone(true);
            // SmartDashboard.putBoolean("climber: overheat", true);
        } else if (!Robot.leftClimber.getDone()) {
            if (!Robot.leftClimber.isBottomLeftSwitchPressed()) {
                // While the front switch is not pressed, keep running the climber Extender Motor out.
                Robot.leftClimber.lower();
            } else {
                Robot.leftClimber.stop();
                Robot.leftClimber.setDone(true);
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
        return Robot.leftClimber.getDone();
    }
}
