package frc.robot.commands.chassis_commands;

import java.util.HashMap;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

import me.wobblyyyy.pathfinder2.control.ProportionalController;
import frc.robot.Robot;

public class CenterShooterToHubCommand extends CommandBase {

    private double yawToHub = 0.0;

    HashMap<String, Double> target;

    final ProportionalController centerShooterController = new ProportionalController(0.1);

    public CenterShooterToHubCommand() {
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void initialize() {
        centerShooterController.reset();
    }


    @Override
    public void execute() {
        target = Robot.shooterCamera.getTargetGoal();
        yawToHub = target.get("Yaw");

        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, centerShooterController.calculate(yawToHub, 0), Rotation2d.fromDegrees(0)));
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Robot.swerveDrive.getGyro()));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(yawToHub) < 0.1;
    }
}