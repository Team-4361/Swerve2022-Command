package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DecreaseAngleCommand extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.adjustor);

        /*
        double currentAngle = Robot.adjustor.getAngle();

        if (currentAngle-5 > 0) {
            Robot.adjustor.setAngle(currentAngle-5);
        } else {
            Robot.adjustor.setAngle(0);
        }
        */
    }

    @Override
    public void execute() {
        Robot.adjustor.lowerAngle(0.3);
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