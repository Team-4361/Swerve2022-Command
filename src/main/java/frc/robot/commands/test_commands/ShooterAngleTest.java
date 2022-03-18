package frc.robot.commands.test_commands;

import java.util.Random;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ShooterAngleTest extends CommandBase {

    private final Random random;
    private Timer timer;
    private boolean timeSet = false;

    public ShooterAngleTest() {
        this.random = new Random();
        this.timer = new Timer();

        timer.reset();
        timer.stop();
    }

    @Override
    public void initialize() {
        addRequirements(Robot.adjustor);
    }

    @Override
    public void execute() {
        // Bounce between different angles to try
        /*
        try {
            for (int i=0; i<3; i++) {
                new IncrementShooterAngle().schedule();
            }
            for (int angle = 80; angle >= 0; angle -= 10) {
                Robot.adjustor.setAngle(angle);
                TimeUnit.SECONDS.sleep(1);
            }
        } catch (InterruptedException ignored) {
        }
        */

        Robot.adjustor.periodic();

        if (timer.get() == 0 && !timeSet) {
            Robot.adjustor.setAngle(random.nextInt(29));
            timer.reset();
            timer.start();
            timeSet = true;
        } else if (timer.get() > 2) {
            timer.stop();
            timeSet = false;
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
