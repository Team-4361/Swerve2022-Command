package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RevShooterCommand extends CommandBase {

    public RevShooterCommand(){
        addRequirements(Robot.shooter);
    }
  
    @Override
    public void initialize() {
       
    }

    
    @Override
    public void execute() {
        Robot.shooter.setShooterWheelVelocity(10);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // Should run forever, so always return false.
        return Robot.shooter.isDesiredSpeed(10);
    }
}