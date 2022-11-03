package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimeDelay extends CommandBase {
    private long duration, msTime;

    public TimeDelay(long ms) {
        msTime = ms;
    }

    @Override
    public void initialize() {
        duration = msTime+System.currentTimeMillis();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis()>=duration;
    }
}
