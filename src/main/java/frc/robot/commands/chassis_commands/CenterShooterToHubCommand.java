package frc.robot.commands.chassis_commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import me.wobblyyyy.pathfinder2.control.ProportionalController;

import java.util.HashMap;

public class CenterShooterToHubCommand extends CommandBase {

    private double yawToHub = 0.0;
    private boolean inRange = false;

    HashMap<String, Double> target;

    final ProportionalController centerShooterController = new ProportionalController(0.015);

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

        double status = target.get("Status");

        if(status != 0){
            inRange = true;
            Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, centerShooterController.calculate(yawToHub, 0), Rotation2d.fromDegrees(0)));
        }else {
            Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0.5, Rotation2d.fromDegrees(0)));
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)));
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(yawToHub) < 0.05) && inRange;
    }
}