package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class DisableIntake extends CommandBase {
    

    @Override
    public void initialize() {
       addRequirements(Robot.intake);
    }

    
    @Override
    public void execute() {
      Robot.intake.retractIntake();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.translateIntake(0);
    }

    @Override
    public boolean isFinished() {
        //Will be finished when they're are no balls in the shooter
        return Robot.intake.getPosition() < (0.1);
    }
}