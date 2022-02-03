package frc.robot.subsystems;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.SHOOTER_PORT;



public class ShooterSubsystem extends SubsystemBase {
  
  private CANSparkMax shooterMotor = new CANSparkMax(SHOOTER_PORT, kBrushless);
  private RelativeEncoder shooterEncoder = shooterMotor.getEncoder();

    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public double getVelocity(){
      return shooterEncoder.getVelocity();
  }

  public boolean isDesiredSpeed(double speed){
      return (shooterEncoder.getVelocity() > speed) ? true : false;
  }

  public void setShooterMotor(double val){
    shooterMotor.set(val);
  }

  public boolean getToDesiredSpeed(double speed){
      if(!isDesiredSpeed(speed)){
        shooterMotor.set(0.8);

        return false;
      }
      else{
        shooterMotor.set(0);

        return true;
      }
  }

}
