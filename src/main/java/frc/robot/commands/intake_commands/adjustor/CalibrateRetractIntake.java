package frc.robot.commands.intake_commands.adjustor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class CalibrateRetractIntake extends CommandBase{
    
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        addRequirements(Robot.intake);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.stopIntakeGroup();
        Robot.intake.resetEncoders();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        if (!Robot.intake.isRetracted()) {
            Robot.intake.retractIntake(0.1);
        } else {
            // The magnet is pressed, stop the intake and end the command.
            Robot.intake.stopIntakeGroup();

            end(false);
        }
    }

    @Override
    public boolean isFinished() {
        return Robot.intake.isRetracted();
    }
}