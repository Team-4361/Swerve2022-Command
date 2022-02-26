package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SpinIntakeOutward extends CommandBase {
    @Override
    public void initialize() {
        addRequirements(Robot.intake);
    }


    @Override
    public void execute() {
        Robot.intake.spinIntakeReject();
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
