package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Climber.R_CLIMBER_ID;

public class RightClimberSubsystem extends AbstractClimberSubsystem {
    public RightClimberSubsystem() {
        super(
                new CANSparkMax(R_CLIMBER_ID, kBrushless)
        );
    }
}
