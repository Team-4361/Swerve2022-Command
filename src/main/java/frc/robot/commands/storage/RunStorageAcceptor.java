package frc.robot.commands.storage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.MotorValue.ACCEPT_SPEED;

public class RunStorageAcceptor extends CommandBase{

    public RunStorageAcceptor(){
        addRequirements(Robot.storage, Robot.intake);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        Robot.storage.setAcceptorMotor(ACCEPT_SPEED);
        Robot.intake.spinAccept();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        Robot.storage.setAcceptorMotor(0);
        Robot.intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}