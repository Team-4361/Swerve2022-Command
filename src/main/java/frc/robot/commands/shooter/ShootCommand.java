package frc.robot.commands.shooter;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.MotorValue.SLOW_ACCEPT_SPEED;

public class ShootCommand extends CommandBase {

    private final int shootRPM;
    public ShootCommand(int shootRPM) {
        this.shootRPM = shootRPM;
    }

    @Override
    public void initialize() {
        addRequirements(Robot.shooter, Robot.storage);
    }

    @Override
    public void execute() {
        if (Robot.shooter.isDesiredSpeed(shootRPM)) {
            Robot.storage.setStorageMotor(1);
            Robot.shooter.setShooterVelocity(shootRPM);
            Robot.storage.setAcceptorMotor(1);
        } else {
            Robot.storage.setStorageMotor(0);
            Robot.storage.setAcceptorMotor(0);
            Robot.shooter.setShooterVelocity(shootRPM);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.storage.setAcceptorMotor(0);
        Robot.storage.setStorageMotor(0);
        Robot.shooter.stopShooter();
    }
}

