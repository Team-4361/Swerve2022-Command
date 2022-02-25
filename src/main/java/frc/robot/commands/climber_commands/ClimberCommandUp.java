package frc.robot.commands.climber_commands;

import frc.robot.Robot;

public class ClimberCommandUp extends AbstractClimberCommand {
    public ClimberCommandUp() {
        super(
                Robot.climber,
                Robot.climber::isTopSwitchPressed,
                Robot.climber::raiseClimber
        );
    }
}
