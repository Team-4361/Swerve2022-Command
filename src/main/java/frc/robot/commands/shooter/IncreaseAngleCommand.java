package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.ShooterAdjustor.ADJUSTOR_ANGLE_MAX;

public class IncreaseAngleCommand extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.adjustor);
        //Robot.adjustor.setAngle(Math.min(Robot.adjustor.getAngle() + 5, ADJUSTOR_ANGLE_MAX));
    }

    @Override
    public void execute() {
        Robot.adjustor.raiseAngle(0.3);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.adjustor.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}