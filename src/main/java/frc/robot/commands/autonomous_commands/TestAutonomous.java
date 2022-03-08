package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import me.wobblyyyy.pathfinder2.Pathfinder;
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
            .setTolerance(2)
            .setAngleTolerance(Angle.fromDeg(5))
            .add(new PointXYZ(0, 0, 0))
            .add(new PointXYZ(0, 0, 0).inDirection(1, Angle.fromDeg(45)))
            .add(new PointXYZ(10, 10, 0))
            .build();
    public static Trajectory LINEAR_TRAJECTORY = new LinearTrajectory(new PointXYZ(5, 5, 0), 0.3, 1, Angle.fromDeg(5));

    private final Pathfinder pathfinder;
    private final Trajectory trajectory;

    public TestAutonomous(PathfinderSubsystem pathfinderSubsystem) {
        super(pathfinderSubsystem, LINEAR_TRAJECTORY);
        this.pathfinder = pathfinderSubsystem.getPathfinder();
        this.trajectory = LINEAR_TRAJECTORY;
    }

    @Override
    public void execute() {
        PointXYZ pos = pathfinder.getPosition();
        SmartDashboard.putString("next marker", trajectory.nextMarker(pos).toString());
        SmartDashboard.putNumber("speed", trajectory.speed(pos));
        SmartDashboard.putString("translation", pathfinder.getTranslation().toString());
        super.execute();
    }
}
