package frc.robot.commands.autonomous;

import frc.robot.Robot;
import frc.robot.commands.shooter.ShootCommand;

public class TimedShootCommand extends ShootCommand {
    private long endTime=0, duration;

    public TimedShootCommand(int shootRPM, long duration) {
        super(shootRPM);
        this.duration = duration*1000;
    }

    @Override
    public void initialize() {
        addRequirements(Robot.shooter);
        endTime = duration + System.currentTimeMillis();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean in) {
        this.endTime = 0;
        super.end(false);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() >= endTime;
    }
}
