package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;

public class ChangeHandMode extends CommandBase {
    
    private boolean finished;

    @Override
    public void initialize() {
       finished = false;
    }

    
    @Override
    public void execute() {
      if(Robot.leftHandedMode == false){
          Robot.leftHandedMode = true;
      } else{
        Robot.leftHandedMode = false;
      }
      finished = true;
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        //Will be finished when they're are no balls in the shooter
        return !finished;
    }
}