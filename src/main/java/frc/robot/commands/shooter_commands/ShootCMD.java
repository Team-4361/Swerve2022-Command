package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.lang.Math;
import java.util.HashMap;

import frc.robot.Robot;


public class ShootCMD extends CommandBase {


    HashMap<String, Double> target;


    public ShootCMD() {
        addRequirements(Robot.shooter, Robot.storage);
    }

    @Override
    public void initialize() {

    }


    @Override
    public void execute() {
        Robot.storage.setStorageMotor(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.shooter.setShooterMotor(0);
        Robot.storage.setStorageMotor(0);
    }

    @Override
    public boolean isFinished() {
        //Will be finished when they are no balls in the shooter
        return (Robot.storage.getBallsLoaded() == 0) ? true : false;
    }
}