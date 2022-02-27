package frc.robot.commands.storage_commands.SequentialStorageCMDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.storage.AcceptColor;

import static frc.robot.Constants.Storage.*;

public class STRSpinIntakeAccept extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.intake, Robot.storage);
    }


    @Override
    public void execute() {
        Robot.intake.spinIntakeAccept();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.stopIntakeGroup();
    }

    @Override
    public boolean isFinished() {
        if(Robot.storage.getProximity() >= PROXIMITY_THRESHOLD){
            if(Robot.storage.getColor().blue > BLUE_THRESHOLD){
                Robot.storage.setCurrentColor(AcceptColor.BLUE);
            } else{
                Robot.storage.setCurrentColor(AcceptColor.RED); 
            }
            return true;
        }
        else{
            return false;
        }
    }
}