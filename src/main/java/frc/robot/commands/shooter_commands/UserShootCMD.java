package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.MotorValue.*;

public class UserShootCMD extends CommandBase{

    private double shootSpeed;

    public UserShootCMD(double shootSpeed){
        this.shootSpeed = shootSpeed;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        addRequirements(Robot.shooter);

        new AutoAdjustShooterAngle().schedule();
    }

    @Override
    public void execute() {
        Robot.shooter.setShooterVelocity(this.shootSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        Robot.storage.setStorageMotor(0);
        Robot.shooter.setShooterMotor(0);
        Robot.shooter.resetPID();
    }
}
