package frc.robot.commands.chassis_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.CubicHermiteSpline;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class MoveToBall extends CommandBase {

    PIDController controller = new PIDController(0.1, 0, 0);

    double currentDistanceToBall = 0;

    public MoveToBall() {

        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void initialize() {
        addRequirements(Robot.swerveDrive);
        currentDistanceToBall = Robot.chassisCamera.getTargetGoal().get("Distance");
    }
    
    @Override
    public void execute() {
        
        currentDistanceToBall = Robot.chassisCamera.getTargetGoal().get("Status") != 0 ? Robot.chassisCamera.getTargetGoal().get("Distance") : currentDistanceToBall;

        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, controller.calculate(currentDistanceToBall), Rotation2d.fromDegrees(0)));
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)));
    }


    @Override
    public boolean isFinished() {
        return currentDistanceToBall < 2;
    }
}