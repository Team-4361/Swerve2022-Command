package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class TimedMoveFWDCMD extends CommandBase {


    private long endTime;

    public TimedMoveFWDCMD() {
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void initialize() {
        endTime = System.currentTimeMillis() + 1500;
    }


    @Override
    public void execute() {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, -0.4, 0, Rotation2d.fromDegrees(0)));
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)));
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() >= endTime;
    }
}