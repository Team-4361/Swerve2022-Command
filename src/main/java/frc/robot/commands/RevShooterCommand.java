package frc.robot.commands;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.MotorValue;
import frc.robot.robotutils.MotorUtil;

public class RevShooterCommand extends CommandBase {

    private double shooterVelocity, shooterCurrent;
    private boolean shootingDone = false;
    private boolean multiBall = true;

    public RevShooterCommand(boolean shootMultiple){
        addRequirements(Robot.shooter);
        multiBall = shootMultiple;
    }
  
    @Override
    public void initialize() {
       SmartDashboard.putString("Shooter: Status", "Idle");
    }

    public void shoot() {
        Robot.shooter.setShooterMotor(MotorUtil.getMotorValue(MotorValue.SHOOT_SPEED, MotorValue.SHOOTER_FLIPPED));

        // Wait until the RPM of the shooter is up to the required speed.
        SmartDashboard.putString("Shooter: Status", "Ramping up");
        while (!Robot.shooter.isDesiredSpeed(MotorValue.SHOOTER_TARGET_RPM)) {
            shooterVelocity = Robot.shooter.getVelocity();
            shooterCurrent = Robot.shooter.getShooterCurrent();
        }

        // Run the Storage Motor until the sensor is not detecting the ball.
        Robot.shooter.setStorageMotor(MotorUtil.getMotorValue(MotorValue.SLOW_ACCEPT_SPEED, MotorValue.STORAGE_FLIPPED));

        while (Robot.shooter.storageSensorCovered()) {
            SmartDashboard.putString("Shooter: Status", "Ball Firing");
        }

        Robot.shooter.setStorageMotor(0);
        Robot.shooter.setShooterMotor(0);

        shootingDone = true;
    }

    public void shootAllBalls() {
        // Detect how many balls are loaded inside the Shooter, and based
        // on this value, shoot either once or twice.
        int ballsLoaded = Robot.shooter.getBallsLoaded();
        shootingDone = false;

        for (int i=0; i<ballsLoaded; i++) {
            shoot();
            while (!shootingDone) {}
        }
    }

    
    @Override
    public void execute() {
        if (multiBall) {
            shootAllBalls();
        } else {
            shoot();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Make sure all of the motors are shut down.
        Robot.shooter.setShooterMotor(0);
        Robot.shooter.setStorageMotor(0);
    }

    @Override
    public boolean isFinished() {
        // Should run forever, so always return false.
        //return Robot.shooter.isDesiredSpeed(10);
        return shootingDone;
    }
}