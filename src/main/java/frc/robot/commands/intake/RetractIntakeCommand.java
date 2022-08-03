package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RetractIntakeCommand extends CommandBase {

    public RetractIntakeCommand() {
        addRequirements(Robot.intakeExtender);
    }

    @Override
    public void initialize() {
        System.out.println("Retracting Intake");
    }

    @Override
    public void execute() {
        Robot.intakeExtender.retractIntake();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intakeExtender.stop();
    }

    @Override
    public boolean isFinished() {
        // Will be finished when the front switch is pressed, meaning all the way extended.
        return Robot.intakeExtender.isRetracted();
    }
}