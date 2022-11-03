package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class ResetGyroCommand extends InstantCommand {
    public ResetGyroCommand() {
        super(() -> Robot.swerveDrive.resetPosition());
    }
}
