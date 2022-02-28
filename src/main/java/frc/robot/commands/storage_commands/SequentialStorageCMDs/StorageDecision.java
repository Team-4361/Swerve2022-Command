package frc.robot.commands.storage_commands.SequentialStorageCMDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.storage.StorageTask;

public class StorageDecision extends CommandBase {
    /**
     * Checks the Task that was determined in {@link IntakeProcessAccept}, and based on the value, schedules either the
     * acceptance command, or reject command. If there are 2 balls loaded in the Storage, then ignore.
     */
    @Override
    public void initialize() {
        StorageTask selectedTask = Robot.storage.getDetectedTask();

        // If any of these are true, the task has ended early or has been interrupted, so we should not continue.
        //assert selectedTask != null && selectedTask != StorageTask.NEUTRAL;
        System.out.println("Making Decision");
        if (selectedTask == StorageTask.ACCEPT) {
            // The acceptance task has been selected, run the STRAcceptBall command.
            System.out.println("Schedueling Accept");
            new StorageAcceptBall().schedule();
        } else if (selectedTask == StorageTask.REJECT) {
            System.out.println("Schedueling Reject");
            new StorageRejectBall().schedule();
        }

        end(false);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.storage.setNextTask(null);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}