package frc.robot.commands.storage_commands.SequentialStorageCMDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.intake_commands.adjustor.RetractIntakeMagnet;
import frc.robot.subsystems.storage.StorageTask;

public class IntakeProcessAccept extends CommandBase {

    private double timeStarted = System.currentTimeMillis();
    private double elapsedTime = 0;

    @Override
    public void initialize() {
        addRequirements(Robot.intake, Robot.storage);
        System.out.println("Intaking");
    }

    /**
     * This is called from {@link ExtendIntakePID}, and is used to listen for the AcceptColor (BLUE/RED), where it
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
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.stopIntakeGroup();

        if(elapsedTime > 10_000){
            new RetractIntakeMagnet().schedule();
        } else{
            new StorageDecision().schedule();
        }
    }

    @Override
    public boolean isFinished() {
        elapsedTime = System.currentTimeMillis() - timeStarted;
        
        return Robot.storage.getNextTask() != null || (elapsedTime > 10_000);
    }
}