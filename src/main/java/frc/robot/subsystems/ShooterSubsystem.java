package frc.robot.subsystems;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.SHOOTER_PORT;



public class ShooterSubsystem extends SubsystemBase {
  
  private CANSparkMax shooterMotor = new CANSparkMax(SHOOTER_PORT, kBrushless);
  private RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
	private PIDController shooterController = new PIDController(0, 1, 0);

  public ShooterSubsystem(){
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public double getVelocity(){
      return shooterEncoder.getVelocity();
  }

  public void setShooterMotor(double val){
    shooterMotor.set(val);
  }

  public void setShooterWheelVelocity(double speed){
    double power = MathUtil.clamp(shooterController.calculate(getVelocity(), speed), -1.0, 1.0);

    shooterMotor.set(power);
  }

  public boolean isDesiredSpeed(double speed){
    return (shooterEncoder.getVelocity() > speed) ? true : false;
  }

  public void resetPID(){
    shooterController.reset();
  }

}
