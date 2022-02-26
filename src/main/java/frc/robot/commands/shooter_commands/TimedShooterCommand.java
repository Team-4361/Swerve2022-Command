package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import java.util.concurrent.TimeUnit;

import static frc.robot.Constants.MotorFlip.SHOOTER_FLIPPED;
import static frc.robot.Constants.MotorFlip.STORAGE_FLIPPED;
import static frc.robot.Constants.MotorValue.SHOOT_SPEED;
import static frc.robot.Constants.MotorValue.SLOW_ACCEPT_SPEED;
import static frc.robot.robot_utils.MotorUtil.getMotorValue;

public class TimedShooterCommand extends CommandBase {
    private boolean finished;

    public TimedShooterCommand() {
        finished = false;
        addRequirements(Robot.shooter);
    }

    @Override
    public void initialize() {
        finished = false;
        new Thread(() -> {
            try {
                if (!finished) {
                    // Start by ramping up the shooter motor to full power and waiting
                    Robot.shooter.setShooterMotor(getMotorValue(SHOOT_SPEED, SHOOTER_FLIPPED));

                    SmartDashboard.putString("Shooter: Status", "Ramping");

                    // wait 3 seconds
                    TimeUnit.SECONDS.sleep(3);

                    SmartDashboard.putString("Shooter: Status", "Shooting");

                    // Start loading the balls into the shooter's reach
                    Robot.shooter.setStorageMotor(getMotorValue(0.4, STORAGE_FLIPPED));

                    // wait 5 seconds to shoot two balls
                    TimeUnit.SECONDS.sleep(5);

                    SmartDashboard.putString("Shooter: Status", "Done");

                    // stop everything
                    Robot.shooter.setShooterMotor(0);
                    Robot.shooter.setStorageMotor(0);

                    finished = true;
                    end(false);
                }
            } catch (Exception ignored) {}
        }).start();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
