package frc.robot.commands.storage_commands.SequentialStorageCMDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.intake_commands.adjustor.RetractIntakeMagnet;

import static frc.robot.Constants.MotorFlip.ACCEPTOR_FLIPPED;
import static frc.robot.robot_utils.MotorUtil.*;

import static frc.robot.Constants.MotorValue.ACCEPT_SPEED;

public class StorageRejectBall extends CommandBase {

    private long timeStarted;

    @Override
    public void initialize() {
        timeStarted = System.currentTimeMillis();
        addRequirements(Robot.intake, Robot.storage);
        System.out.println("Rejecting Ball");
    }

    @Override
    public void execute() {
        Robot.intake.spinIntakeReject();
        Robot.storage.setAcceptorMotor(-getMotorValue(ACCEPT_SPEED, ACCEPTOR_FLIPPED));

        if (isFinished()) {
            end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.stopIntakeGroup();
        Robot.storage.setAcceptorMotor(0);

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
    }

    @Override
    public boolean isFinished() {
        return getElapsedTime() >= 2000;
    }

    private double getElapsedTime() {
        return System.currentTimeMillis() - timeStarted;
    }
}