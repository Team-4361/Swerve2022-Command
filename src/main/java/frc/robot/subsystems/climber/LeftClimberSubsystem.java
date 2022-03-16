package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Climber;
import frc.robot.Constants.MotorFlip;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class LeftClimberSubsystem extends AbstractClimberSubsystem {
    public LeftClimberSubsystem() {
        super(
                new DigitalInput(Climber.BL_LIMIT_ID),
                new DigitalInput(Climber.TL_LIMIT_ID),
                new CANSparkMax(Climber.L_CLIMBER_ID, kBrushless),
                MotorFlip.CLIMBER_LEFT_FLIPPED
        );
    }

    public boolean isBottomLeftSwitchPressed() {
        return bottomLimit.get();
    }

    public boolean isTopLeftSwitchPressed() {
        return topLimit.get();
    }
}
