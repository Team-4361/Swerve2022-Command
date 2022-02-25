package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Climber.*;
import static frc.robot.robot_utils.MotorUtil.*;
import static frc.robot.Constants.MotorValue.*;
import static frc.robot.Constants.MotorFlip.*;

public class ClimberSubsystem extends SubsystemBase {

    private final CANSparkMax leftClimberMTR = new CANSparkMax(L_CLIMBER_ID, kBrushless);
    private final CANSparkMax rightClimberMTR = new CANSparkMax(R_CLIMBER_ID, kBrushless);

    private final DigitalInput blSwitch, brSwitch, tlSwitch, trSwitch;

    private final CANSparkMax[] climberGroup = new CANSparkMax[]{leftClimberMTR, rightClimberMTR};

    public ClimberSubsystem() {
        blSwitch = new DigitalInput(BL_LIMIT_ID);
        brSwitch = new DigitalInput(BR_LIMIT_ID);
        tlSwitch = new DigitalInput(TL_LIMIT_ID);
        trSwitch = new DigitalInput(TR_LIMIT_ID);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void moveClimberUp() {
        if (!tlSwitch.get() && !trSwitch.get()) {
            runMotors(climberGroup, getMotorValue(CLIMBER_SPEED, CLIMBER_FLIPPED));
        }
    }

    public void moveClimberDown() {
        if (!blSwitch.get() && !brSwitch.get()) {
            runMotors(climberGroup, getMotorValue(-CLIMBER_SPEED, CLIMBER_FLIPPED));
        }
    }

    public void stopClimber() {
        stopMotors(climberGroup);
    }

    // public void lowerClimber() {
    //     runMotors(climberGroup, getMotorValue(-CLIMBER_SPEED, CLIMBER_FLIPPED));

    //     while (!blSwitch.get() && !brSwitch.get()) {
    //         Thread.onSpinWait();
    //     }

    //     stopMotors(climberGroup);
    // }

    // public void raiseClimber() {
    //     runMotors(climberGroup, getMotorValue(CLIMBER_SPEED, CLIMBER_FLIPPED));

    //     while (!tlSwitch.get() && !trSwitch.get()) {
    //         Thread.onSpinWait();
    //     }

    //     stopMotors(climberGroup);
    // }

    public boolean isTransTopClear(){
        return !tlSwitch.get() || !trSwitch.get();
    }

    public boolean isTranBtmClear(){
        return !blSwitch.get() || !brSwitch.get();
    }
}
