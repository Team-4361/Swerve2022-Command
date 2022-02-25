package frc.robot.commands.climber_commands;

import frc.robot.Robot;

public class ClimberCommandDown extends AbstractClimberCommand {
    public ClimberCommandDown() {
        super(
                Robot.climber,
                Robot.climber::isBottomSwitchPressed,
                Robot.climber::lowerClimber
        );
    }
}
