package frc.robot.subsystems;

import java.util.HashMap;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.SwerveChassis;

public class SwerveDriveSubsystem extends SubsystemBase {
  
  private SwerveChassis swerveChassis;
  public AHRS gyro;

  public SwerveDriveSubsystem() {
    swerveChassis = new SwerveChassis();
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();
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

  public Rotation2d getGyro(){
    return gyro.getRotation2d();
  }

  public void resetGyro(){
    gyro.reset();
  }
}
