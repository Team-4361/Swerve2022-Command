package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;


public class RetractIntake extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.intake);
    }

    @Override
    public void execute() {
        // This runs repeatedly until the command is ended.
        if (!Robot.intake.isRearSwitchPressed()) {
            // While the rear switch is not pressed, keep running the Intake Retract Motor out.
            Robot.intake.retractIntake();
        } else {
            // The limit switch is pressed, stop the intake and end the command.
            Robot.intake.stopIntakeGroup();
            end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // The stop intake method disables retract motors, as well as the accept/reject
        // motors. This is okay in this scenario because we don't want to be taking in balls when
        // the intake is retracted.
        Robot.intake.stopIntakeGroup();
    }

    @Override
    public boolean isFinished() {
        // Will be finished when the front switch is pressed, meaning all the way extended.
        return Robot.intake.isRearSwitchPressed();
    }
}