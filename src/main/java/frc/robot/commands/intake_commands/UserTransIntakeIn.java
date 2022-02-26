package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class UserTransIntakeIn extends CommandBase {
    @Override
    public void initialize() {
        addRequirements(Robot.intake);
    }


    @Override
    public void execute() {
        if (Robot.intake.getAveragePosition() > 0.05)
            Robot.intake.retractIntake();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.stopIntakeGroup();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
