package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.AngleAdjustSubsystem;
import me.wobblyyyy.pathfinder2.geometry.Angle;

import java.util.Map;

import static frc.robot.Constants.Shooter.SHOOTER_WHEEL_RADIUS;
import static frc.robot.Robot.shooter;

public class AutoShootCommand extends SequentialCommandGroup {
    protected static class AdjustAngleCommand extends CommandBase {
        private static final double TOLERANCE = 2;
        private final AngleAdjustSubsystem adjustor = Robot.adjustor;
        private final Angle angle;

        public AdjustAngleCommand(Angle angle) {
            this.angle = angle;

            addRequirements(adjustor);
        }

        @Override
        public void execute() {
            adjustor.setAngle(angle.deg());

            if (isFinished()) {
                end(false);
            }
        }

        @Override
        public boolean isFinished() {
            return adjustor.atDesiredAngle(angle.deg(), TOLERANCE);
        }
    }

    protected static class ShootBallCommand extends CommandBase {
        private final double requiredVelocity;

        public ShootBallCommand(Map<String, Double> map) {
            this.requiredVelocity = (calculateVelocity(
                    map.get("Pitch"),
                    map.get("Distance"),
                    map.get("Yaw")
            ) * SHOOTER_WHEEL_RADIUS * Math.PI) / 30;

            addRequirements(shooter);
        }

        @Override
        public void initialize() {
            // Run the SensorShootCommand with the calculated velocity, after setting all the angles.
            new SensorShootCommand(requiredVelocity).schedule();

            end(false);
        }
        
        private static double calculateVelocity(double pitchToTarget,
                                                double distanceToTarget,
                                                double initialHeight) {
            pitchToTarget = Math.toRadians(pitchToTarget);

            return (1 / Math.cos(pitchToTarget)) * Math.sqrt((((
                                Math.pow(distanceToTarget, 2) + 
                                (2.44 * distanceToTarget) + 1.4884) * 9.80)) 
                    / (2 * (2.64 - initialHeight - (Math.tan(pitchToTarget) 
                                * (distanceToTarget + 1.22)))));
        }

        @Override
        public boolean isFinished() {
            return shooter.getBallsLoaded() == 0;
        }
    }

    public AutoShootCommand() {
        super(
                new AdjustAngleCommand(Angle.fromDeg(45)),
                new ShootBallCommand(Robot.shooterCamera.getTargetGoal())
        );
    }
}
