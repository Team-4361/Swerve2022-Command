package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

import static frc.robot.Constants.MotorValue.SLOW_ACCEPT_SPEED;

public class ShootCommand extends CommandBase {
    private double shootSpeed;

    public ShootCommand(double shootSpeed) {
        this.shootSpeed = shootSpeed;
    }

    public ShootCommand() {
        this.shootSpeed = SmartDashboard.getNumber("Shoot Speed", 4500);
    }

    @Override
    public void initialize() {
        addRequirements(Robot.shooter, Robot.storage);
    }

    @Override
    public void execute() {
        if (Robot.shooter.isDesiredSpeed(shootSpeed)) {
            Robot.storage.setStorageMotor(SLOW_ACCEPT_SPEED);
            Robot.shooter.setShooterVelocity(shootSpeed);
        } else {
            Robot.storage.setStorageMotor(0);
            Robot.shooter.setShooterVelocity(shootSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.storage.setAcceptorMotor(0);
        Robot.storage.setStorageMotor(0);
        Robot.shooter.stopShooter();
    }
}
