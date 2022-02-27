package frc.robot.commands.storage_commands.SequentialStorageCMDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.commands.intake_commands.RetractIntake;

import frc.robot.commands.intake_commands.RetractIntake;

import static frc.robot.Constants.MotorValue.ACCEPT_SPEED;


public class STRAcceptBall extends CommandBase {

    private int ballsLoaded = 0;

    @Override
    public void initialize() {
        addRequirements(Robot.storage);
        ballsLoaded = Robot.storage.getBallsLoaded();
    }


    @Override
    public void execute() {
        Robot.storage.setAcceptorMotor(ACCEPT_SPEED);
        Robot.storage.setStorageMotor(ACCEPT_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.storage.setAcceptorMotor(0);
        Robot.storage.setStorageMotor(0);
        
        CommandScheduler.getInstance().schedule(new RetractIntake());
    }

    @Override
    public boolean isFinished() {
        switch(ballsLoaded){
            case 0:
                return Robot.storage.frontProximityCovered();
            case 1:
                return Robot.storage.rearProximityCovered();
            case 2:
                CommandScheduler.getInstance().schedule(new STRRejectBall());
                return true;
        }
        return false;
    }
}

