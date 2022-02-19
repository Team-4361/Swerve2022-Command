package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RevDecreaseShooterAngle extends CommandBase {
    private final double runAngle;

    /**
     * Sets the shooter to decrease angle by a certain amount using a specialized Command.
     * @param decrease Angle in Degrees to decrease (raise down) by.
     */
    public RevDecreaseShooterAngle(double decrease) {
        this.runAngle = Robot.shooter.getAdjustAngle() - decrease;

        addRequirements(Robot.shooter);
    }

    @Override
    public void execute() {
        Robot.shooter.setAdjustAngle(runAngle);
    }

    @Override
    public boolean isFinished() {
        // ± 5 degrees tolerance, can be adjusted later.
        return Robot.shooter.atDesiredAngle(runAngle, 5);
    }
}
