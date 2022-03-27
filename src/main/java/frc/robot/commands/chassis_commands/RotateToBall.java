package frc.robot.commands.chassis_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.CubicHermiteSpline;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RotateToBall extends CommandBase {

    PIDController controller = new PIDController(0.1, 0, 0);

    double currentYawToBall = 0;

    public RotateToBall() {

        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void initialize() {
        addRequirements(Robot.swerveDrive);
        currentYawToBall = Robot.chassisCamera.getTargetGoal().get("Yaw");
    }
    
    @Override
    public void execute() {
        currentYawToBall = Robot.chassisCamera.getTargetGoal().get("Yaw");

        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, controller.calculate(currentYawToBall), Rotation2d.fromDegrees(0)));
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)));
    }


    @Override
    public boolean isFinished() {
        return currentYawToBall < 2;
    }
}