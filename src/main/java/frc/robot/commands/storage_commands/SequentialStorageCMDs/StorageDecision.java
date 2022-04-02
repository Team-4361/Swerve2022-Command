package frc.robot.commands.storage_commands.SequentialStorageCMDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.intake_commands.adjustor.RetractIntakeMagnet;
import frc.robot.subsystems.storage.AcceptColor;
import frc.robot.subsystems.storage.StorageTask;
import static frc.robot.Constants.MotorValue.ACCEPT_SPEED;

import static frc.robot.Constants.Storage.RETRACT_ON_ACCEPT;

/**
 * Checks the Task that was determined in {@link IntakeProcessAccept}, and based on the value, schedules either the
 * acceptance command, or reject command. If there are 2 balls loaded in the Storage, then ignore.
 */
public class StorageDecision extends CommandBase {
    private void acceptBall() {
        System.out.println("Storage: Scheduling Accept...");

        if (RETRACT_ON_ACCEPT) { new RetractIntakeMagnet().schedule(); }

        new StorageAcceptBall().andThen(new StorageRunAcceptor(ACCEPT_SPEED, 2)).schedule();
    }

    private void rejectBall() {
        System.out.println("Storage: Scheduling Reject...");
        new StorageRejectBall().schedule();
    }

    @Override
    public void initialize() {
        StorageTask selectedTask = Robot.storage.getDetectedTask();

        // If any of these are true, the task has ended early or has been interrupted, so we should not continue.
        //assert selectedTask != null && selectedTask != StorageTask.NEUTRAL;
        System.out.println("Storage: Making Decision");

        // If the acceptance color is set to neutral, always attempt to take in the ball no matter if it should
        // be accepted or rejected.
        switch (selectedTask) {
            case ACCEPT: this.acceptBall(); break;
            case REJECT: {
                if (Robot.storage.getAcceptColor() == AcceptColor.NEUTRAL) {
                    this.acceptBall();
                } else {
                    this.rejectBall();
                }
                break;
            }
            case NEUTRAL: break;
        }
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