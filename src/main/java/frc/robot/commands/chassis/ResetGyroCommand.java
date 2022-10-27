package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.utils.motor.MotorUtil;

public class ResetGyroCommand extends InstantCommand {
    public ResetGyroCommand() {
        super(() -> Robot.swerveDrive.resetGyro());
    }
}
