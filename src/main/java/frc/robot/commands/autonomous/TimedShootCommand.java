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
    }

    @Override
    public void execute() {
        super.execute();
        if (endTime==0) {
            this.endTime = System.currentTimeMillis() + duration;
        }
    }

    @Override
    public void end(boolean in) {
        this.endTime = 0;
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() >= endTime;
    }
}
