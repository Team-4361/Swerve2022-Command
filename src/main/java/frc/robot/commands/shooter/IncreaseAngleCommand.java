package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class IncreaseAngleCommand extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.adjustor);
        //Robot.adjustor.setAngle(Math.min(Robot.adjustor.getAngle() + 5, ADJUSTOR_ANGLE_MAX));
    }

    @Override
    public void execute() {
        Robot.adjustor.increaseAdjustMotor(0.3);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.adjustor.stopAdjustMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}