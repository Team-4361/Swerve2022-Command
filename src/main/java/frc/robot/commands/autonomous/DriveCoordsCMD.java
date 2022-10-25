package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.utils.motor.MotorUtil.inTolerance;

public class DriveCoordsCMD extends CommandBase {
    private final PIDController driveController;
    private final double xMeters, yMeters, twistAngle;
    private Pose2d pose;

    public DriveCoordsCMD(double xMeters, double yMeters, double twistAngle) {
        this.driveController = new PIDController(0.05, 0, 0);

        this.xMeters = xMeters;
        this.yMeters = yMeters;
        this.twistAngle = twistAngle;
    }

    public DriveCoordsCMD(double twistAngle) {
        this(Robot.swerveDrive.getPose().getX(), Robot.swerveDrive.getPose().getY(), twistAngle);
    }

    @Override
    public void initialize() {
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void execute() {
        pose = Robot.swerveDrive.getPose();

        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                clamp(driveController.calculate(pose.getX(), xMeters), -0.5, 0.5),
                clamp(driveController.calculate(pose.getY(), yMeters), -0.5, 0.5),
                clamp(driveController.calculate(Robot.swerveDrive.getRobotHeading().getDegrees(), twistAngle), -0.5, 0.5),
                new Rotation2d(0)
        ));
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return inTolerance(xMeters, pose.getX(), 0.5) && inTolerance(yMeters, pose.getY(), 0.5) &&
                inTolerance(twistAngle, Robot.swerveDrive.getRobotHeading().getDegrees(), 1);
    }
}
