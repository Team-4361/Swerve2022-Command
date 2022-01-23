package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase {
  
  private SwerveChassis swerveChassis;

  public SwerveDriveSubsystem() {
    swerveChassis = new SwerveChassis();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public HashMap<String, SwerveModuleState> getSwerveModuleStates(){
    return swerveChassis.getSwerveModuleStates();
  }

  public void drive(ChassisSpeeds speeds){
    swerveChassis.drive(speeds);
  }
}
