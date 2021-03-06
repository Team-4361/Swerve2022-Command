package frc.robot.commands.test_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ChassisDriveTest extends CommandBase {

    private final boolean moveRight = true;
    private boolean moveLeft, moveFoward, moveBackward;

    private final PIDController chassisController;

    public ChassisDriveTest() {
        chassisController = new PIDController(0.1, 0, 0);
        addRequirements(Robot.swerveDrive);
    }

    private void driveRight() {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.8, 0, 0, Rotation2d.fromDegrees(0)));
    }

    private void driveLeft() {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-0.8, 0, 0, Rotation2d.fromDegrees(0)));
    }

    private void driveFWD() {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0.8, 0, Rotation2d.fromDegrees(0)));
    }

    private void driveLBack() {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, -0.8, 0, Rotation2d.fromDegrees(0)));
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Drive forward while the command is being executed.
        driveFWD();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}