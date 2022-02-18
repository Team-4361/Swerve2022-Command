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
        // Try different angles to run the shooter at
        try {
            for (int angle=0; angle<=80; angle+=10) {
                Robot.shooter.setAdjustAngle(angle);
                TimeUnit.SECONDS.sleep(1);
            }
            for (int angle=80; angle>=0; angle-=10) {
                Robot.shooter.setAdjustAngle(angle);
                TimeUnit.SECONDS.sleep(1);
            }
        } catch (InterruptedException ex) {}
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
