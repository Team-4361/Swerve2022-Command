package frc.robot.commands.storage_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.MotorValue.ACCEPT_SPEED;

public class RunStorageCMD extends CommandBase{

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        addRequirements(Robot.storage);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        Robot.storage.setStorageMotor(ACCEPT_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        Robot.storage.setStorageMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
