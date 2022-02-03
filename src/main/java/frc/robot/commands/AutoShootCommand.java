package frc.robot.commands;

import static frc.robot.Constants.INTAKE_MOTOR_SPEED;
import static frc.robot.Constants.ACCEPTOR_MOTOR_SPEED;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCommand extends CommandBase {
    
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  
    @Override
    public void initialize() {
       
    }

    
    @Override
    public void execute() {
       calculateDesiredVelocity(pitchToTarget, distanceToTarget)
    }

    @Override
    public void end(boolean interrupted) {
       
    }

    @Override
    public boolean isFinished() {
        // Should run forever, so always return false.
        return false;
    }

    public double calculateDesiredVelocity(double pitchToTarget, double distanceToTarget){
        return 0.0;
    }
}