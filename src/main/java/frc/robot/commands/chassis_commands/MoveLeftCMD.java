package frc.robot.commands.chassis_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import java.util.HashMap;

public class MoveLeftCMD extends CommandBase {


    private PIDController chassisController;

    private final double distanceToMove = 2.4;
    private double initDriveEncoderDist = 0;


    public MoveLeftCMD(){
        chassisController = new PIDController(0.1, 0, 0);
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void initialize() {
        initDriveEncoderDist = Robot.swerveDrive.getDistance();
    }

    
    @Override
    public void execute() {
        double power = chassisController.calculate(getAbsDistance(), distanceToMove+0.1);

        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-power, 0, 0, Rotation2d.fromDegrees(0)));
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)));
    }

    @Override
    public boolean isFinished() {
        return (getAbsDistance() > distanceToMove) ? true : false;
    }

    private double getAbsDistance(){
        return Robot.swerveDrive.getDistance() - initDriveEncoderDist;
    }
}