package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RevShooterCMD extends CommandBase {
    
    private double targetRPM  = 0;

    public RevShooterCMD(double targetRPM){
        this.targetRPM = targetRPM;
    }

    @Override
    public void execute() {
        if(Robot.storage.getBallsLoaded() >= 1){
            Robot.shooter.setShooterVelocity(targetRPM);
        }
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        addRequirements(Robot.shooter);

    }

    @Override
    public void end(boolean interrupted) {
        Robot.shooter.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
