package frc.robot.commands.intake_commands.adjustor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class CalibrateRetractIntake extends CommandBase{

    public CalibrateRetractIntake(){
        addRequirements(Robot.intakeExtender);
    }
    
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intakeExtender.stop();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        if (!Robot.intakeExtender.isFullyRetracted()) {
            Robot.intakeExtender.retractIntake(0.1);
        } else {
            // The magnet is pressed, stop the intake and end the command.
            Robot.intakeExtender.stop();

            end(false);
        }
    }

    @Override
    public boolean isFinished() {
        return Robot.intakeExtender.isFullyRetracted();
    }
}