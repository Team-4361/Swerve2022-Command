package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.chassis.ResetGyroCommand;
import frc.robot.commands.chassis.SwerveDriveMode;

public class AutoCommandGroup extends SequentialCommandGroup {
    public AutoCommandGroup() {
        super(
                // Resets the gyroscope to zero and matches Robot-relative
                new ResetGyroCommand(SwerveDriveMode.ROBOT_RELATIVE),

                // Drives the robot forward 1.5 meters in order to clear the launchpad.
                new DriveCoordsCMD(0, 1.5, 0),

                // Start shooting for around 7 seconds to dump the balls.
                new TimedShootCommand(4500, 7),

                // Rotate the robot 180 degrees to line up with field-oriented
                new DriveCoordsCMD(180),

                // Finally, reset the gyroscope and enable field-oriented to drive.
                new ResetGyroCommand(SwerveDriveMode.FIELD_RELATIVE)
        );
    }
}
