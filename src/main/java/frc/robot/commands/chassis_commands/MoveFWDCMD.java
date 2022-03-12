package frc.robot.commands.chassis_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class MoveFWDCMD extends CommandBase {


    private double timeStarted;

    public MoveFWDCMD() {
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void initialize() {
        timeStarted = System.currentTimeMillis();
        addRequirements(Robot.swerveDrive);
    }


    @Override
    public void execute() {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0.4, 0, Rotation2d.fromDegrees(0)));
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)));
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - timeStarted) > 2000;
    }
}