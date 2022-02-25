package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ExtendIntake extends CommandBase {

    @Override
    public void initialize() {
       addRequirements(Robot.intake);
    }
    
    @Override
    public void execute() {
        // This runs repeatedly until the command is ended.
        if (!Robot.intake.isFrontSwitchPressed()) {
            // While the front switch is not pressed, keep running the Intake Extender Motor out.
            Robot.intake.extendIntake();
        } else {
            // The limit switch is pressed, stop the intake and end the command.
            Robot.intake.stopIntakeGroup();
            end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.stopIntakeGroup();
    }

    @Override
    public boolean isFinished() {
        // Will be finished when the front switch is pressed, meaning all the way extended.
        return Robot.intake.isFrontSwitchPressed();
    }
}