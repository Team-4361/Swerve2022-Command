package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class TimedMoveFWDCMD extends CommandBase {


    private long endTime, startTime;

    public TimedMoveFWDCMD() {

    }

    @Override
    public void initialize() {
        addRequirements(Robot.swerveDrive);

        // hopefully gives the gyro a chance to zero out.
        Robot.swerveDrive.autoDrive(0,0,0);

        endTime = System.currentTimeMillis() + 2500;
        startTime = System.currentTimeMillis() + 1000;

    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() > startTime) {
            Robot.swerveDrive.autoDrive(0,0.4,0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.autoDrive(0,0,0);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() >= endTime;
    }
}