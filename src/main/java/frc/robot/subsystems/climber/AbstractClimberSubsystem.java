package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.motor.MotorUtil;

import static frc.robot.Constants.MotorFlip.CLIMBER_FLIPPED;
import static frc.robot.Constants.MotorValue.CLIMBER_SPEED;

public class AbstractClimberSubsystem extends SubsystemBase {
    private final CANSparkMax climberMotor;

    /**
     * Creates a new {@link AbstractClimberSubsystem} using the Climber Motor.
     * @param climberMotor The {@link CANSparkMax} motor to use for translating the Climber.
     */
    public AbstractClimberSubsystem(CANSparkMax climberMotor) {
        this.climberMotor = climberMotor;
    }

    /** Raises the Climber Motor. */
    public void raise() {
        climberMotor.set(MotorUtil.flip(CLIMBER_SPEED, CLIMBER_FLIPPED));
    }

    /** Lowers the Climber Motor. */
    public void lower() {
        climberMotor.set(MotorUtil.flip(-CLIMBER_SPEED, CLIMBER_FLIPPED));
    }

    /** Stops the Climber Motor. */
    public void stop() {
        climberMotor.stopMotor();
    }
}
