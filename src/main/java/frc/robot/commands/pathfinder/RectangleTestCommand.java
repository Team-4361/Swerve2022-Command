package frc.robot.commands.pathfinder;

import java.util.ArrayList;
import java.util.List;

import frc.robot.commands.PathfinderCommand;
import me.wobblyyyy.pathfinder2.Pathfinder;
import me.wobblyyyy.pathfinder2.geometry.Angle;
import me.wobblyyyy.pathfinder2.geometry.PointXYZ;
import me.wobblyyyy.pathfinder2.prebuilt.TrajectoryFactory;
import me.wobblyyyy.pathfinder2.trajectory.Trajectory;
import me.wobblyyyy.pathfinder2.trajectory.multi.segment.MultiSegmentTrajectory;

public class RectangleTestCommand extends PathfinderCommand {
    private static final double TOLERANCE = 2.0;
    private static final Angle ANGLE_TOLERANCE = Angle.fromDeg(5);
    private static final double SPEED = 0.25;
    private static final List<PointXYZ> POINTS = new ArrayList<>() {{
        add(new PointXYZ(0, 0, 0));
        add(new PointXYZ(0, 10, 0));
        add(new PointXYZ(10, 10, 0));
        add(new PointXYZ(10, 0, 0));
        add(new PointXYZ(0, 0, 0));
    }};
    private static final List<Trajectory> TRAJECTORIES = 
        TrajectoryFactory.getLinearTrajectories(
            POINTS, 
            SPEED, 
            TOLERANCE, 
            ANGLE_TOLERANCE
        );
    private static final Trajectory TRAJECTORY = 
        new MultiSegmentTrajectory(TRAJECTORIES);

    public RectangleTestCommand(Pathfinder pathfinder) {
        super(pathfinder, TRAJECTORY);
    }
}
