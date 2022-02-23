package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

import static frc.robot.Constants.Storage.LENGTH_ROD_TO_ANGULAR_POS;

public class EnableIntake extends CommandBase {
    

    @Override
    public void initialize() {
       addRequirements(Robot.intake);
    }

    
    @Override
    public void execute() {
      Robot.intake.extendIntake();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.translateIntake(0);
    }

    @Override
    public boolean isFinished() {
        //Will be finished when they are no balls in the shooter
        return Robot.intake.getPosition() > (LENGTH_ROD_TO_ANGULAR_POS - 0.1);
    }
}