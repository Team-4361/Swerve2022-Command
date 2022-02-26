package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.robot_utils.MotorUtil;

import java.util.concurrent.TimeUnit;

import static frc.robot.Constants.MotorFlip;
import static frc.robot.Constants.MotorValue;

public class RevShooterCommand extends CommandBase {
    private double shooterVelocity, shooterCurrent, autoVelocity = 0;
    private int shootAngle = -1;
    private volatile boolean shootingDone = false, realDone = false;
    private boolean multiBall = true;

    public RevShooterCommand(boolean shootMultiple) {
        addRequirements(Robot.shooter);
        multiBall = shootMultiple;
    }

    /**
     * Used for autonomous shooting
     */
    public RevShooterCommand(boolean shootMultiple, double velocity) {
        addRequirements(Robot.shooter);
        multiBall = shootMultiple;

        // This is designed to be run under velocity, NOT motor speed.
        autoVelocity = velocity;
    }

    public RevShooterCommand setShootAngle(int angle) {
        shootAngle = angle;
        return this;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Shooter: Status", "Idle");
    }

    public void shoot(boolean reset) {
        try {
            SmartDashboard.putString("Shooter: Status", "Ramping up");

            if (shootAngle > -1) {
                // Set the shooting angle
                Robot.adjustor.setAngle(shootAngle);
            }

            if (autoVelocity == 0) {
                // This is running the command in the Full Power Shoot mode, non-autonomous
                Robot.shooter.setShooterMotor(MotorUtil.getMotorValue(MotorValue.SHOOT_SPEED, MotorFlip.SHOOTER_FLIPPED));

                while (!Robot.shooter.isDesiredSpeed(MotorValue.SHOOTER_TARGET_RPM)) {
                    Thread.onSpinWait();
                }
            } else {
                Robot.shooter.setShooterVelocity(autoVelocity);

                while (!Robot.shooter.isDesiredSpeed(autoVelocity)) {
                    Thread.onSpinWait();
                }
            }

            // Run the Storage Motor until the sensor is not detecting the ball.
            Robot.shooter.setStorageMotor(MotorUtil.getMotorValue(MotorValue.SLOW_ACCEPT_SPEED, MotorFlip.STORAGE_FLIPPED));

            while (Robot.shooter.storageSensorCovered()) {
                SmartDashboard.putString("Shooter: Status", "Ball Firing");
            }

            TimeUnit.MILLISECONDS.sleep(1000);

            Robot.shooter.setStorageMotor(0);

            // If disabled, prevents the motor from completey spinning down, saving time from having to
            // spin back up after each run.
            if (reset) {
                Robot.shooter.setShooterMotor(0);
            }

            SmartDashboard.putString("Shooter: Status", "Idle");

            shootingDone = true;
        } catch (InterruptedException ignored) {
        }
    }

    public void shootAllBalls() {
        // Detect how many balls are loaded inside the Shooter, and based
        // on this value, shoot either once or twice.
        int ballsLoaded = Robot.shooter.getBallsLoaded();
        shootingDone = false;

        for (int i = 0; i < ballsLoaded; i++) {
            shoot(false);
            while (!shootingDone) {
                Thread.onSpinWait();
            }
        }
    }


    @Override
    public void execute() {
        if (multiBall) {
            shootAllBalls();
        } else {
            shoot(true);
        }
        realDone = true;
        end(false);
    }

    @Override
    public void end(boolean interrupted) {
        // Make sure all the motors are shut down.
        Robot.shooter.setShooterMotor(0);
        Robot.shooter.setStorageMotor(0);
    }

    @Override
    public boolean isFinished() {
        return realDone;
    }
}
