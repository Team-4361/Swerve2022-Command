package frc.robot.commands.chassis_commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ArcadeCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final Supplier<ChassisSpeeds> chassisSpeedsFunction;

    public ArcadeCommand(Supplier<ChassisSpeeds> chassisSpeedsFunction) {

        this.chassisSpeedsFunction = chassisSpeedsFunction;
        Robot.swerveDrive.resetGyro();

        addRequirements(Robot.swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ChassisSpeeds speeds = chassisSpeedsFunction.get();

        Robot.swerveDrive.drive(speeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Robot.swerveDrive.getGyro()));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
