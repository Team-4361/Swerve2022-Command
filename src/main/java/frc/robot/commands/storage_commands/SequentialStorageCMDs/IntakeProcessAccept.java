package frc.robot.commands.storage_commands.SequentialStorageCMDs;

import java.io.Console;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.storage.StorageTask;

public class IntakeProcessAccept extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.intake, Robot.storage);
        System.out.println("Intaking");
    }

    /**
     * This is called from {@link StorageExtendIntake}, and is used to listen for the AcceptColor (BLUE/RED), where it
     * can be sequenced to call Accept/Deny based off it.
     */
    @Override
    public void execute() {
        Robot.intake.spinIntakeAccept();

        // Check and wait for a proper task to be determined, either accepting or denying. Store this value,
        // and then exit the command.
        StorageTask task = Robot.storage.getDetectedTask();

        if (task != StorageTask.NEUTRAL) {
            // A proper task has been determined, save the value and end.
            Robot.storage.setNextTask(task);
            //end(false);
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.stopIntakeGroup();
    }

    @Override
    public boolean isFinished() {
        return Robot.storage.getNextTask() != null;
    }
}