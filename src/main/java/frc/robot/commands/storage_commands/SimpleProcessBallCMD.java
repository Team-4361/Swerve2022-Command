package frc.robot.commands.storage_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.commands.intake_commands.adjustor.ExtendIntakeMagnet;
import frc.robot.commands.intake_commands.adjustor.RetractIntakeMagnet;

import static frc.robot.Constants.MotorValue.ACCEPT_SPEED;

public class SimpleProcessBallCMD extends CommandBase {

    public SimpleProcessBallCMD(){
        addRequirements(Robot.intake, Robot.storage);  
    }
    @Override
    public void initialize() {
        new ExtendIntakeMagnet().schedule();
    }

    @Override
    public void execute() {
        
        if(Robot.storage.getBallsLoaded() == 0){
            Robot.storage.setStorageMotor(ACCEPT_SPEED);
        } else {
            Robot.storage.setStorageMotor(0);
        }
        Robot.storage.setAcceptorMotor(ACCEPT_SPEED);
        Robot.intake.spinIntakeAccept();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.stopIntakeGroup();
        Robot.storage.setAcceptorMotor(0);
        Robot.storage.setStorageMotor(0);
        
        new RetractIntakeMagnet().schedule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}