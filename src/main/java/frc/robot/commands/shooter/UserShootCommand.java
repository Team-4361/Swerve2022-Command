package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.MotorValue.SLOW_ACCEPT_SPEED;

public class UserShootCommand extends ShootCommand {
    public UserShootCommand() {
        super(5500);
    }
}
