package frc.robot.commands.storage_commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.intake_commands.RetractIntake;
import frc.robot.subsystems.storage.TaskListener;
import me.wobblyyyy.pathfinder2.scheduler.Task;

import java.util.concurrent.TimeUnit;

public class ProcessBallCommand extends CommandBase {
    private static int ballsLoaded = 0;
    private enum State { ACCEPTING, DENYING, NEUTRAL }

    protected static State currentState = State.NEUTRAL;
    protected static int oldBalls;

    public static TaskListener defaultListener = new TaskListener() {
        @Override
        public void onAcceptTask() {
            oldBalls = Robot.storage.getBallsLoaded();
            currentState = State.ACCEPTING;
            SmartDashboard.putString("Storage Status", "Accepting");
        }

        @Override
        public void onRejectTask() {
            oldBalls = Robot.storage.getBallsLoaded();
            currentState = State.DENYING;
            SmartDashboard.putString("Storage Status", "Rejecting");
        }

        @Override public void onTimeout() {}
    };

    private void reject() {
        new Thread(() -> {
            try {
                Robot.storage.spinStorageReject();
                Robot.storage.spinIndexReject(true);

                TimeUnit.SECONDS.sleep(2);

                end(false);
            } catch (InterruptedException ignored) {}
        }).start();
    }

    private void accept() {
        int currentBallsLoaded = Robot.storage.getBallsLoaded();

        switch (oldBalls) {
            case 0: {
                if (currentBallsLoaded <= oldBalls) {
                    // keep running the motors until the rear proximity is covered
                    Robot.storage.spinStorageAccept();
                    Robot.storage.spinIndexAccept(true);
                } else {
                    end(false);
                }
                break;
            }

            case 1: {
                if (currentBallsLoaded <= oldBalls) {
                    // keep running the index motor until front proximity is covered
                    Robot.storage.spinStorageAccept();
                } else {
                    end(false);
                }
                break;
            }

            default: {
                end(false);
            }
        }
    }

    public ProcessBallCommand() {
        addRequirements(Robot.storage, Robot.intake);
        currentState = State.NEUTRAL;
    }

    @Override
    public void initialize() {
        // The intake motor should always be running while the robot is active.
        Robot.intake.spinIntakeAccept();
        Robot.storage.setRunningState(true);
    }

    @Override
    public void execute() {
        // This is run in a constant loop, and it will run these checking
        // methods when waiting for a ball response.
        switch (currentState) {
            case ACCEPTING:
                accept();
                break;

            case DENYING:
                reject();
                break;

            case NEUTRAL:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.storage.setRunningState(false);
        Robot.storage.translateAdjustorMotor(0);
        Robot.storage.stopIntakeMotor();
        Robot.storage.stopIndexMotor();

        currentState = State.NEUTRAL;
    }

    @Override
    public boolean isFinished() {
        // We should return true when the ball has been successfully processed,
        // or has been timed out. Otherwise, the command is still running and
        // return false.
        return !Robot.storage.getRunningState();
    }
}