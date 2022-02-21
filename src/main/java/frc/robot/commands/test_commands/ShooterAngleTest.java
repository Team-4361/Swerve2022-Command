package frc.robot.commands.test_commands;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ShooterAngleTest extends CommandBase {

    public ShooterAngleTest() {
        addRequirements(Robot.shooter);
    }

    @Override
    public void execute() {
        // Bounce between different angles to try
        try {
            for (int angle=0; angle<=80; angle+=10) {
                Robot.adjustor.setAngle(angle);
                TimeUnit.SECONDS.sleep(1);
            }
            for (int angle=80; angle>=0; angle-=10) {
                Robot.adjustor.setAngle(angle);
                TimeUnit.SECONDS.sleep(1);
            }
        } catch (InterruptedException ex) {}
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
