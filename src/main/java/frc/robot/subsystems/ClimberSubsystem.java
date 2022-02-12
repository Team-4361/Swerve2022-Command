package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Climber.L_CLIMBER_PORT;
import static frc.robot.Constants.Climber.R_CLIMBER_PORT;
import static frc.robot.Constants.Climber.B_CLIMBER_SWITCH;
import static frc.robot.Constants.Climber.T_CLIMBER_SWITCH;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class ClimberSubsystem extends SubsystemBase {
  
  private CANSparkMax leftClimberMTR = new CANSparkMax(L_CLIMBER_PORT, kBrushless);
  private CANSparkMax rightClimberMTR = new CANSparkMax(R_CLIMBER_PORT, kBrushless);

  private DigitalInput bottomProxSwitch = new DigitalInput(B_CLIMBER_SWITCH);
  private DigitalInput topProxSwitch = new DigitalInput(T_CLIMBER_SWITCH);

  public ClimberSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void moveClimberUp() {
    if(!topProxSwitch.get()){
      leftClimberMTR.set(0.5);
      rightClimberMTR.set(0.5);
    }
  }

  public void moveClimberDown() {
    if(!bottomProxSwitch.get()){
      leftClimberMTR.set(-0.5);
      rightClimberMTR.set(-0.5);
    }
  }

  public void stopClimber(){
    leftClimberMTR.set(0);
    rightClimberMTR.set(0);
  }

}
