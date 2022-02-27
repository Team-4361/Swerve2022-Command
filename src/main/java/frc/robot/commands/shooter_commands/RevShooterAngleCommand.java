package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RevShooterAngleCommand extends CommandBase {
    private final double runAngle;

    public RevShooterAngleCommand(double angle) {
        this.runAngle = angle;

        addRequirements(Robot.shooter);
    }

    @Override
    public void execute() {
        Robot.adjustor.setAngle(runAngle);
    }

    @Override
    public boolean isFinished() {
        return Robot.adjustor.atDesiredAngle(runAngle, 5);
    }
}
