package frc.robot.commands.autonomous_commands;

import me.wobblyyyy.pathfinder2.geometry.Angle;
import me.wobblyyyy.pathfinder2.geometry.PointXYZ;
import me.wobblyyyy.pathfinder2.trajectory.LinearTrajectory;
import me.wobblyyyy.pathfinder2.trajectory.Trajectory;
import me.wobblyyyy.pathfinder2.trajectory.spline.AdvancedSplineTrajectoryBuilder;
import me.wobblyyyy.pathfinder2.wpilib.PathfinderSubsystem;
import me.wobblyyyy.pathfinder2.wpilib.TrajectoryCommand;

public class TestAutonomous extends TrajectoryCommand {
    public static Trajectory MOVE_FORWARDS = new LinearTrajectory(
            new PointXYZ(0, 10, 0), 0.5, 2, Angle.fromDeg(5));
    public static Trajectory SPLINE_TRAJECTORY = new AdvancedSplineTrajectoryBuilder()
            .setStep(0.1)
            .setSpeed(0.3)
            .setTolerance(0.05)
            .setAngleTolerance(Angle.fromDeg(5))
            .add(new PointXYZ(0, 0, 0))
            .add(new PointXYZ(0, 0, 0).inDirection(1, Angle.fromDeg(45)))
            .add(new PointXYZ(10, 10, 0))
            .build();

    public TestAutonomous(PathfinderSubsystem pathfinderSubsystem) {
        super(pathfinderSubsystem, SPLINE_TRAJECTORY);
    }
}
