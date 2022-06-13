package frc.robot.commands.storage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.intake.ExtendIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;

import static frc.robot.Constants.MotorValue.ACCEPT_SPEED;

/**
 * This {@link ProcessBallCommand} is designed to handle processing the accepting ball, from extending
 * out the Intake, spinning the mechanism, and then retracting it when finished.
 */
public class ProcessBallCommand extends CommandBase {
    public ProcessBallCommand() {
        addRequirements(Robot.intake, Robot.storage);
    }

    @Override
    public void initialize() {
        // Schedule the ExtendIntakeMagnet command to make fully extend the intake before
        // attempting.
        new ExtendIntakeCommand().schedule();
    }

    @Override
    public void execute() {
        if (Robot.storage.getBallsLoaded() == 0) {
            Robot.storage.setStorageMotor(ACCEPT_SPEED);
        } else {
            Robot.storage.setStorageMotor(0);
        }
        Robot.storage.setAcceptorMotor(ACCEPT_SPEED);
        Robot.intake.spinAccept();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.stop();
        Robot.storage.setAcceptorMotor(0);
        Robot.storage.setStorageMotor(0);

        new RetractIntakeCommand();
    }

    @Override
    public boolean isFinished() { return false; }
}
