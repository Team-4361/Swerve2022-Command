package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeShooter.L_INTAKE_MOTOR_ID;
import static frc.robot.Constants.IntakeShooter.R_INTAKE_MOTOR_ID;
import static frc.robot.Constants.IntakeShooter.INTAKE_MOTOR_ID;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class IntakeSubsystem extends SubsystemBase {
  
  private CANSparkMax leftIntakeTransMTR = new CANSparkMax(L_INTAKE_MOTOR_ID, kBrushless);
  private CANSparkMax rightIntakeTransMTR = new CANSparkMax(R_INTAKE_MOTOR_ID, kBrushless);
  private CANSparkMax intakeMTR = new CANSparkMax(INTAKE_MOTOR_ID, kBrushless);
  private RelativeEncoder leftIntakeTransEncoder;

  public IntakeSubsystem() {
    leftIntakeTransEncoder = leftIntakeTransMTR.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void moveIntakeOut() {
      leftIntakeTransMTR.set(0.5);
      rightIntakeTransMTR.set(0.5);
  }

  public void moveIntakeIn() {
    leftIntakeTransMTR.set(-0.5);
    rightIntakeTransMTR.set(-0.5);
  }

  public void runIntakeIn(){
    intakeMTR.set(0.5);
  }

  public void runIntakeOut(){
    intakeMTR.set(-0.5);
  }

  public void stopIntake(){
    leftIntakeTransMTR.set(0);
    rightIntakeTransMTR.set(0);

    intakeMTR.set(0);
  }

  public double getPosition(){
    return leftIntakeTransEncoder.getPosition();
  }

}