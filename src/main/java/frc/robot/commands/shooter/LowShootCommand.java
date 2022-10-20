package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LowShootCommand extends SequentialCommandGroup {
    public LowShootCommand() {
        super(
                new SetAngleCommand(30),
                new ShootCommand(2500)
        );
    }
}
