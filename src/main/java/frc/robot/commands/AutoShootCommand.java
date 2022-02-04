package frc.robot.commands;

import static frc.robot.Constants.NETWORK_TABLE_HOSTNAME;
import static frc.robot.Constants.CAMERA_NAME;
import static frc.robot.Constants.CAMERA_HEIGHT;
import static frc.robot.Constants.CAMERA_PITCH;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.lang.Math;
import java.util.HashMap;

import  frc.robot.ShooterVisionCamera;

public class AutoShootCommand extends CommandBase {
    
    private ShooterSubsystem shooter;

		private ShooterVisionCamera camera;

    HashMap<String, Double> target; 


		public AutoShootCommand(){
			shooter = new ShooterSubsystem();
			camera = new ShooterVisionCamera(NETWORK_TABLE_HOSTNAME, CAMERA_NAME, CAMERA_HEIGHT, CAMERA_PITCH);
		}
  
    @Override
    public void initialize() {
       
    }

    
    @Override
    public void execute() {
      target = camera.getTargetGoal();
      double requiredShooterVelocity = calculateDesiredVelocity(target.get("Pitch"), target.get("Distance"), target.get("Yaw"));

      while(!shooter.isDesiredSpeed(requiredShooterVelocity)){
        shooter.setShooterWheelVelocity(requiredShooterVelocity);
      }
			 
    }

    @Override
    public void end(boolean interrupted) {
       
    }

    @Override
    public boolean isFinished() {
        // Should run forever, so always return false.
        return false;
    }

    public double calculateDesiredVelocity(double pitchToTarget, double distanceToTarget, double initialHeight){
			pitchToTarget = Math.toRadians(pitchToTarget);

      return (1/Math.cos(pitchToTarget)) * Math.sqrt((((Math.pow(distanceToTarget, 2) + (2.44*distanceToTarget) + 1.4884)*9.80))/(2*(2.64-initialHeight-(Math.tan(pitchToTarget)*(distanceToTarget+1.22)))));
    }
}