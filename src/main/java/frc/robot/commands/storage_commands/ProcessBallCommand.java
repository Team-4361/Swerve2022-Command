package frc.robot.commands.storage_commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.intake_commands.RetractIntake;
import frc.robot.subsystems.storage.TaskListener;

import java.util.concurrent.TimeUnit;

public class ProcessBallCommand extends CommandBase {
    private boolean finished = false, accepting = false, rejecting = false;

    private int ballsLoaded = 0;

    private void stop() {
        Robot.storage.setRunningState(false);
        Robot.storage.translateAdjustorMotor(0);
        Robot.storage.stopIntakeMotor();
        Robot.storage.stopIndexMotor();

        end(false);
    }

    private void reject() {
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    Robot.storage.spinStorageReject();
                    Robot.storage.spinIndexReject(true);

                    TimeUnit.SECONDS.sleep(2);

                    stop();
                } catch (InterruptedException e) {}
            }
        }).start();
    }

    private void accept() {
        switch (ballsLoaded) {
            case 0: {
                if (!Robot.storage.rearProximityCovered()) {
                    // keep running the motors until the rear proximity is covered
                    Robot.storage.spinStorageAccept();
                    Robot.storage.spinIndexAccept(true);
                } else {
                    stop();
                }
            }

            case 1: {
                if (!Robot.storage.frontProximityCovered()) {
                    // keep running the index motor until front proximity is covered
                    Robot.storage.spinStorageAccept();
                } else {
                    stop();
                }
            }

            default: {
                stop();
            }
        }
    }

    public ProcessBallCommand() {
        addRequirements(Robot.storage);
    }

    @Override
    public void initialize() {
        // The intake motor should always be running while the robot is active.
        Robot.storage.addListener(new TaskListener() {
            @Override
            public void onAcceptTask() {
                ballsLoaded = Robot.storage.getBallsLoaded();
                accepting = true;
            }

            @Override
            public void onRejectTask() {
                ballsLoaded = Robot.storage.getBallsLoaded();
                rejecting = true;
            }

            @Override
            public void onTimeout() {
                stop();
            }
        });

        Robot.storage.setRunningState(true);
    }

    @Override
    public void execute() {
        if (accepting) {
            accept();
        } else if (rejecting) {
            reject();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // If the command ends, possibly turn off the motor?
        stop();

        // TODO: we may not want to immediately retract everything when command is done.
        new RetractIntake().execute();
    }

    @Override
    public boolean isFinished() {
        // We should return true when the ball has been successfully processed,
        // or has been timed out. Otherwise, the command is still running and
        // return false.
        return finished;
    }
}