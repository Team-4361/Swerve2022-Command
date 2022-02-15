package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import me.wobblyyyy.pathfinder2.Pathfinder;
import me.wobblyyyy.pathfinder2.geometry.Translation;
import me.wobblyyyy.pathfinder2.trajectory.Trajectory;

public class PathfinderCommand extends CommandBase {
    private final Pathfinder pathfinder;
    private final Trajectory trajectory;

    public PathfinderCommand(Pathfinder pathfinder,
                             Trajectory trajectory) {
        this.pathfinder = pathfinder;
        this.trajectory = trajectory;
    }

    @Override
    public void initialize() {
        pathfinder.followTrajectory(trajectory);
    }

    @Override
    public void execute() {
        pathfinder.tick();
    }

    @Override
    public void end(boolean wasInterrupted) {
        pathfinder.clear();
        pathfinder.setTranslation(Translation.ZERO);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
