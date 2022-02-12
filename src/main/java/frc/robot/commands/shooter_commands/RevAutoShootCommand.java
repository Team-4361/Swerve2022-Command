package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.lang.Math;
import java.util.HashMap;

import frc.robot.Robot;

public class RevAutoShootCommand extends CommandBase {
    

    HashMap<String, Double> target; 


		public RevAutoShootCommand(){
      addRequirements(Robot.shooter, Robot.storage);
		}
  
    @Override
    public void initialize() {
       
    }

    
    @Override
    public void execute() {
      target = Robot.camera.getTargetGoal();

      double requiredShooterVelocity = calculateDesiredVelocity(target.get("Pitch"), target.get("Distance"), target.get("Yaw"));

      // TODO: will not work, change
      Robot.shooter.setShooterMotor(requiredShooterVelocity);
    }

    @Override
    public void end(boolean interrupted) {
       Robot.shooter.setShooterMotor(0);
    }

    @Override
    public boolean isFinished() {
        //Will be finished when they're are no balls in the shooter
        return false;
    }

    public double calculateDesiredVelocity(double pitchToTarget, double distanceToTarget, double initialHeight){
			pitchToTarget = Math.toRadians(pitchToTarget);

      return (1/Math.cos(pitchToTarget)) * Math.sqrt((((Math.pow(distanceToTarget, 2) + (2.44*distanceToTarget) + 1.4884)*9.80))/(2*(2.64-initialHeight-(Math.tan(pitchToTarget)*(distanceToTarget+1.22)))));
    }
}