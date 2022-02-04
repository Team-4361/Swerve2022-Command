package frc.robot.commands;

import static frc.robot.Constants.INTAKE_MOTOR_SPEED;
import static frc.robot.Constants.ACCEPTOR_MOTOR_SPEED;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ShooterVisionCamera;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

import static frc.robot.Constants.NETWORK_TABLE_HOSTNAME;

import java.util.HashMap;

import static frc.robot.Constants.CAMERA_NAME;
import static frc.robot.Constants.CAMERA_HEIGHT;
import static frc.robot.Constants.CAMERA_PITCH;

public class CenterShooterToHubCommand extends CommandBase {
    
    private final SwerveDriveSubsystem swerve = new SwerveDriveSubsystem();
    private ShooterVisionCamera camera; 

    HashMap<String, Double> target;

    @Override
    public void initialize() {
        camera = new ShooterVisionCamera(NETWORK_TABLE_HOSTNAME, CAMERA_NAME, CAMERA_HEIGHT, CAMERA_PITCH);
    }

    
    @Override
    public void execute() {
        target = camera.getTargetGoal();
        double yaw = target.get("Yaw");
        
        while(Math.abs(yaw) > 0.08){
            if(yaw > 0){
                swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0.5, Rotation2d.fromDegrees(0)));
            }
            else if(yaw < 0){
                swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, -0.5, Rotation2d.fromDegrees(0)));
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)));
    }

    @Override
    public boolean isFinished() {
        // Should run forever, so always return false.
        return false;
    }
}