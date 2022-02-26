package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.AngleAdjustSubsystem;
import me.wobblyyyy.pathfinder2.geometry.Angle;

public class ShootCommand extends SequentialCommandGroup {
    private static class AdjustAngleCommand extends CommandBase {
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
        }

        @Override
        public boolean isFinished() {
            return adjustor.atDesiredAngle(angle.deg(), TOLERANCE);
        }
    }

    public ShootCommand() {
        super(
                new AdjustAngleCommand(Angle.fromDeg(45))
        );
    }
}
