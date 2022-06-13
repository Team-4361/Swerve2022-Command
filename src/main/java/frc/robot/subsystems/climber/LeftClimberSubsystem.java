package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Climber.L_CLIMBER_ID;

public class LeftClimberSubsystem extends AbstractClimberSubsystem {
    public LeftClimberSubsystem() {
        super(
                new CANSparkMax(L_CLIMBER_ID, kBrushless)
        );
    }
}
