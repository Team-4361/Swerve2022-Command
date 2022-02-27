package frc.robot.commands.storage_commands.SequentialStorageCMDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;

public class STRDecide extends CommandBase {

    @Override
    public void initialize() {
        if(Robot.storage.getCurrentColor() == Robot.storage.getAcceptColor() && Robot.storage.getBallsLoaded() != 2){
            CommandScheduler.getInstance().schedule(new STRAcceptBall());
        }else{
            CommandScheduler.getInstance().schedule(new STRRejectBall());
        }
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}