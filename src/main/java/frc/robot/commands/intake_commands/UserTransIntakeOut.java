package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.Intake.INTAKE_EXTEND_ROTATIONS;

public class UserTransIntakeOut extends CommandBase {
    @Override
    public void initialize() {
        addRequirements(Robot.intake);
    }


    @Override
    public void execute() {
        if (Robot.intake.getAveragePosition() < INTAKE_EXTEND_ROTATIONS)
            Robot.intake.extendIntake();
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
