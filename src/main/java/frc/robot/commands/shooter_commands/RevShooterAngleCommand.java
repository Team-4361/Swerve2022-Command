package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RevShooterAngleCommand extends CommandBase {
    private final double runAngle;

    /**
     * Sets the shooter to a specified angle using a specialized Command.
     * @param angle Angle in Degrees to be set to.
     */
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
        // ± 5 degrees tolerance, can be adjusted later.
        return Robot.adjustor.atDesiredAngle(runAngle, 5);
    }
}
