package frc.robot.commands.storage_commands.SequentialStorageCMDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.intake_commands.adjustor.RetractIntakeLimit;

import static frc.robot.Constants.MotorFlip.ACCEPTOR_FLIPPED;
import static frc.robot.Constants.MotorFlip.STORAGE_FLIPPED;
import static frc.robot.Constants.MotorValue.ACCEPT_SPEED;
import static frc.robot.robot_utils.MotorUtil.*;


public class StorageAcceptBall extends CommandBase {
    private int ballsLoaded = 0;

    @Override
    public void initialize() {
        addRequirements(Robot.storage);
        ballsLoaded = Robot.storage.getBallsLoaded();
        System.out.println("Accepting Ball");
    }


    @Override
    public void execute() {
        // Depending on how many balls are currently loaded, either turn on both the Acceptor and Storage
        // motors, or just turn on the front Acceptor motor to prevent the 2nd ball from being pushed out
        // the top.
        switch (ballsLoaded) {
            case 0:
                if (Robot.storage.frontProximityCovered()) {
                    end(false);
                }
                System.out.println("Storing 1 ball");
                // There are no balls loaded currently, it is safe to run both motors.
                Robot.storage.setAcceptorMotor(getMotorValue(ACCEPT_SPEED, ACCEPTOR_FLIPPED));
                Robot.storage.setStorageMotor(getMotorValue(ACCEPT_SPEED, STORAGE_FLIPPED));

                break;
            case 1:
                System.out.println("Storing 2 balls");
                if (Robot.storage.rearProximityCovered()) {
                    end(false);
                }

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
        Robot.storage.setAcceptorMotor(0);
        Robot.storage.setStorageMotor(0);

        switch (Robot.storage.getRetractMode()) {
            case RETRACT_ALWAYS:
                new RetractIntakeLimit().schedule();
                break;

            case RETRACT_WHEN_FULL:
                if (Robot.storage.isFull()) {
                    new RetractIntakeLimit().schedule();
                }
                break;

            default:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        switch (ballsLoaded) {
            case 0:
                return Robot.storage.frontProximityCovered();
            case 1:
                return Robot.storage.rearProximityCovered();
            default:
                return false;
        }
    }
}

