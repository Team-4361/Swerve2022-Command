package frc.robot.commands;

import static frc.robot.Constants.NETWORK_TABLE_HOSTNAME;
import static frc.robot.Constants.CAMERA_NAME;
import static frc.robot.Constants.CAMERA_HEIGHT;
import static frc.robot.Constants.CAMERA_PITCH;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import java.lang.Math;

import  frc.robot.ShooterVisionCamera;

public class AutoShootCommand extends CommandBase {
    
    private ShooterSubsystem shooter;
		private SwerveDriveSubsystem swerveChassis;

		private ShooterVisionCamera camera; 


		public AutoShootCommand(ShooterSubsystem shooter, SwerveDriveSubsystem swerveChassis){
			this.shooter = shooter;
			this.swerveChassis = swerveChassis;
			
			camera = new ShooterVisionCamera(NETWORK_TABLE_HOSTNAME, CAMERA_NAME, CAMERA_HEIGHT, CAMERA_PITCH);
		}
  
    @Override
    public void initialize() {
       
    }

    
    @Override
    public void execute() {
       double requiredShooterVelocity = calculateDesiredVelocity(pitchToTarget, distanceToTarget);
			 
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

      return (1/Math.cos(pitchToTarget)) * Math.sqrt((((Math.Pow(distanceToTarget, 2) + (2.44*distanceToTarget) + 1.4884)*9.80))/(2(2.64-initialHeight-(Math.tan(pitchToTarget)*(distanceToTarget+1.22)))));
    }
}