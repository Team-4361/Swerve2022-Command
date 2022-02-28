package frc.robot.commands.storage_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.commands.intake_commands.ExtendIntake;
import frc.robot.commands.intake_commands.RetractIntake;
import static frc.robot.Constants.MotorValue.ACCEPT_SPEED;

public class SimpleProcessBallCMD extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.intake, Robot.storage);
        CommandScheduler.getInstance().schedule(new ExtendIntake());
    }

    @Override
    public void execute() {
        Robot.storage.setAcceptorMotor(ACCEPT_SPEED);
        Robot.storage.setStorageMotor(ACCEPT_SPEED);
        Robot.intake.spinIntakeAccept();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.stopIntakeGroup();
        Robot.storage.setAcceptorMotor(0);
        Robot.storage.setStorageMotor(0);
        CommandScheduler.getInstance().schedule(new RetractIntake());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}