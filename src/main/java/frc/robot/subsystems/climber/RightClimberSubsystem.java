package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Climber;
import frc.robot.Constants.MotorFlip;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class RightClimberSubsystem extends AbstractClimberSubsystem {
    public RightClimberSubsystem() {
        super(
                new DigitalInput(Climber.BR_LIMIT_ID),
                new DigitalInput(Climber.TR_LIMIT_ID),
                new CANSparkMax(Climber.R_CLIMBER_ID, kBrushless),
                MotorFlip.CLIMBER_RIGHT_FLIPPED
        );
    }

    public boolean isBottomRightSwitchPressed() {
        return bottomLimit.get();
    }

    public boolean isTopRightSwitchPressed() {
        return topLimit.get();
    }
}
