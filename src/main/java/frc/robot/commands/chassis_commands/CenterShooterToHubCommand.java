package frc.robot.commands.chassis_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import java.util.HashMap;

public class CenterShooterToHubCommand extends CommandBase {
    
    private double yawToHub = 0.0;

    HashMap<String, Double> target;

    PIDController centerShooterController;

    public CenterShooterToHubCommand(){
        centerShooterController = new PIDController(0.1, 0, 0);
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void initialize() {
        centerShooterController.reset();
    }

    
    @Override
    public void execute() {
        target = Robot.camera.getTargetGoal();
        yawToHub = target.get("Yaw");
        
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, centerShooterController.calculate(yawToHub, 0), Rotation2d.fromDegrees(0)));
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Robot.swerveDrive.getGyro()));
    }

    @Override
    public boolean isFinished() {
      if(Math.abs(yawToHub) < 0.1){
        return true;
      } else { 
        return false;
      }
    }
}