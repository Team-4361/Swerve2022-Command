package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.utils.motor.MotorUtil.inTolerance;

public class SetAngleCommand extends CommandBase {
    private final double targetAngle;

    public SetAngleCommand(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    @Override
    public void initialize() {
        addRequirements(Robot.adjustor);
        Robot.adjustor.setAngle(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
