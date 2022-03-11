package frc.robot.commands.storage_commands.SequentialStorageCMDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.intake_commands.adjustor.ExtendIntakeMagnet;
import frc.robot.commands.intake_commands.adjustor.RetractIntakeMagnet;

import static frc.robot.Constants.MotorFlip.ACCEPTOR_FLIPPED;
import static frc.robot.robot_utils.motor.MotorUtil.*;

import static frc.robot.Constants.MotorValue.ACCEPT_SPEED;

public class StorageRejectBall extends CommandBase {

    private long timeStarted;
    private boolean extended = false;

    @Override
    public void initialize() {
        timeStarted = System.currentTimeMillis();
        this.extended = false;

        addRequirements(Robot.intake, Robot.storage);
        System.out.println("Rejecting Ball");

        // Retract the intake while rejecting to hopefully bind it better.
        new RetractIntakeMagnet().schedule();
    }

    @Override
    public void execute() {
        Robot.intake.spinIntakeReject();
        Robot.storage.setAcceptorMotor(-getMotorValue(ACCEPT_SPEED, ACCEPTOR_FLIPPED));

        if (getElapsedTime() > 500 && !extended) {
            this.extended = true;
            new ExtendIntakeMagnet().schedule();
        }

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
        return getElapsedTime() >= 3000;
    }

    private double getElapsedTime() {
        return System.currentTimeMillis() - timeStarted;
    }
}