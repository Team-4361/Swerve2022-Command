package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.Intake.INTAKE_EXTEND_SETPOINT;

import frc.robot.Robot;

public class UserTransIntakeOut extends CommandBase {
    @Override
    public void initialize() {
        addRequirements(Robot.intake);
    }


    @Override
    public void execute() {
        if (Robot.intake.getAveragePosition() < INTAKE_EXTEND_SETPOINT)
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
