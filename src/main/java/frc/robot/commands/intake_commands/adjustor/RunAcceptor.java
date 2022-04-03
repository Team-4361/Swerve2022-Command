package frc.robot.commands.intake_commands.adjustor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RunAcceptor extends CommandBase{

    public RunAcceptor(){
        addRequirements(Robot.intake);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.spinIntakeAccept();
    }

    @Override
    public void execute() {
        Robot.intake.stopIntakeGroup();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}