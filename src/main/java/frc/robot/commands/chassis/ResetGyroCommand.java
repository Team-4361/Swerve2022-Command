package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.utils.motor.MotorUtil;

public class ResetGyroCommand extends CommandBase {
    private final SwerveDriveMode updateDriveMode;

    @Override
    public void initialize() {
        addRequirements(Robot.swerveDrive);
        Robot.swerveDrive.resetGyro();
        Robot.swerveDrive.setDriveMode(updateDriveMode);
    }

    public ResetGyroCommand() {
        this(Robot.swerveDrive.getDriveMode());
    }

    public ResetGyroCommand(SwerveDriveMode updateDriveMode) {
        this.updateDriveMode = updateDriveMode;
    }

    @Override
    public boolean isFinished() {
        return MotorUtil.inTolerance(0, Robot.swerveDrive.getGyro().getDegrees(), 1);
    }
}
