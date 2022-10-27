package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimeDelayMS extends CommandBase {
    private long endTime, durationMS;

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() >= endTime;
    }

    @Override
    public void initialize() {
        this.endTime = System.currentTimeMillis() + durationMS;
    }

    public TimeDelayMS(long ms) {
        this.durationMS = ms;
    }
}
