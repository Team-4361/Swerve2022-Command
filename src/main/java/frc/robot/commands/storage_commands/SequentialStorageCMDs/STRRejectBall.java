package frc.robot.commands.storage_commands.SequentialStorageCMDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.commands.intake_commands.RetractIntake;

import static frc.robot.Constants.MotorValue.ACCEPT_SPEED;

public class STRRejectBall extends CommandBase {

    private double timeStarted = System.nanoTime();

    @Override
    public void initialize() {
        addRequirements(Robot.intake, Robot.storage);
    }


    @Override
    public void execute() {
        Robot.intake.spinIntakeReject();
        Robot.storage.setAcceptorMotor(-ACCEPT_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.stopIntakeGroup();
        Robot.storage.setAcceptorMotor(0);

        CommandScheduler.getInstance().schedule(new RetractIntake());
    }

    @Override
    public boolean isFinished() {
        if(getElapsedTime() > 2000000000){
            return true;
        } else{
            return false;
        }
    }

    private double getElapsedTime(){
        return System.nanoTime() - timeStarted;
    }
}