package frc.robot.commands.storage_commands.SequentialStorageCMDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.intake_commands.adjustor.RetractIntakeMagnet;

import static frc.robot.Constants.MotorFlip.ACCEPTOR_FLIPPED;
import static frc.robot.Constants.MotorFlip.STORAGE_FLIPPED;
import static frc.robot.Constants.MotorValue.ACCEPT_SPEED;
import static frc.robot.Constants.Storage.STORAGE_EXTRA_TIME_MS;
import static frc.robot.robot_utils.MotorUtil.*;


public class StorageAcceptBall extends CommandBase {
    private int ballsLoaded = 0;
    private long startTimeMillis = -1;
    private long endTimeMillis = -1;

    @Override
    public void initialize() {
        addRequirements(Robot.storage);
        ballsLoaded = Robot.storage.getBallsLoaded();
        startTimeMillis = -1;
        System.out.println("Accepting Ball");
    }

    private void timeEnd() {
        if (startTimeMillis == -1) {
            this.startTimeMillis = System.currentTimeMillis();
            this.endTimeMillis = startTimeMillis + STORAGE_EXTRA_TIME_MS;
        } else {
            if (System.currentTimeMillis() > endTimeMillis) {
                end(false);
            }
        }
    }

    @Override
    public void execute() {
        // Depending on how many balls are currently loaded, either turn on both the Acceptor and Storage
        // motors, or just turn on the front Acceptor motor to prevent the 2nd ball from being pushed out
        // the top.
        switch (ballsLoaded) {
            case 0:
                // if (Robot.storage.frontProximityCovered()) {
                //     timeEnd();
                // }
                System.out.println("Storing 1 ball");
                // There are no balls loaded currently, it is safe to run both motors.
                Robot.storage.setAcceptorMotor(getMotorValue(ACCEPT_SPEED, ACCEPTOR_FLIPPED));
                Robot.storage.setStorageMotor(getMotorValue(ACCEPT_SPEED, STORAGE_FLIPPED));

                break;
            case 1:
                System.out.println("Storing 2 balls");
                // if (Robot.storage.rearProximityCovered()) {
                //     timeEnd();
                // }

                // There is already a ball loaded, we should only run the front acceptor motor.
                Robot.storage.setAcceptorMotor(getMotorValue(ACCEPT_SPEED, ACCEPTOR_FLIPPED));
                Robot.storage.setStorageMotor(getMotorValue(ACCEPT_SPEED, STORAGE_FLIPPED));

                break;
            default:
                // There is most likely 2 balls loaded, if the command gets this far through, hopefully a problem
                // will be avoided.
                end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.storage.setStorageMotor(0);

        switch (Robot.storage.getRetractMode()) {
            case RETRACT_ALWAYS:
                new RetractIntakeMagnet().schedule();
                break;

            case RETRACT_WHEN_FULL:
                if (Robot.storage.isFull()) {
                    new RetractIntakeMagnet().schedule();
                }
                break;

            default:
                break;
        }

        new StorageRunAcceptor(ACCEPT_SPEED, 0.3).schedule();
    }

    @Override
    public boolean isFinished() {
        
            
        switch (ballsLoaded) {
            case 0:
                return Robot.storage.frontProximityCovered(); //&& System.currentTimeMillis() > endTimeMillis;
            case 1:
                return Robot.storage.rearProximityCovered();// && System.currentTimeMillis() > endTimeMillis;
            default:
                return false;
        }
    }
}

