package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Climber.*;
import static frc.robot.Constants.MotorFlip.CLIMBER_FLIPPED;
import static frc.robot.Constants.MotorValue.CLIMBER_SPEED;
import static frc.robot.robot_utils.MotorUtil.*;

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
    }

    public void stopClimber() {
        stopMotors(climberGroup);
    }

    public void lowerClimber() {
        translateClimber(-CLIMBER_SPEED);
    }

    public void raiseClimber() {
        translateClimber(CLIMBER_SPEED);
    }

    public void translateClimber(double value) {
        runMotors(climberGroup, getMotorValue(value, CLIMBER_FLIPPED));
    }

    /**
     * @return If the top switch is pressed
     */
    public boolean isTopSwitchPressed() {
        return tlSwitch.get() && trSwitch.get();
    }

    /**
     * @return If the bottom switch is pressed
     */
    public boolean isBottomSwitchPressed() {
        return blSwitch.get() && brSwitch.get();
    }
}
