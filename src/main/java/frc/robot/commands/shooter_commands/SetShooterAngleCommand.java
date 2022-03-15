package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SetShooterAngleCommand extends CommandBase {
    private final double runAngle;

    public SetShooterAngleCommand(double angle) {
        this.runAngle = angle;

        addRequirements(Robot.shooter);
    }

    @Override
    public void execute() {
        Robot.adjustor.setTargetAngle(runAngle);
    }

    @Override
    public boolean isFinished() {
        return Robot.adjustor.atDesiredAngle(runAngle);
    }
}
