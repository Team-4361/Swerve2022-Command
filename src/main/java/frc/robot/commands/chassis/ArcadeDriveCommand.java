package frc.robot.commands.chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import java.util.function.Supplier;

@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
public class ArcadeDriveCommand extends CommandBase {
    private final Supplier<ChassisSpeeds> chassisSpeedsFunction;

    public ArcadeDriveCommand(Supplier<ChassisSpeeds> chassisSpeedsFunction) {
        this.chassisSpeedsFunction = chassisSpeedsFunction;
        Robot.swerveDrive.resetGyro();

        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = chassisSpeedsFunction.get();
        Robot.swerveDrive.drive(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
