package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RevIncreaseShooterAngle extends CommandBase {
    private final double runAngle;

    /**
     * Sets the shooter to increase angle by a certain amount using a specialized Command.
     * @param increase Angle in Degrees to increase (raise up) by.
     */
    public RevIncreaseShooterAngle(double increase) {
        this.runAngle = Robot.shooter.getAdjustAngle() + increase;

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
