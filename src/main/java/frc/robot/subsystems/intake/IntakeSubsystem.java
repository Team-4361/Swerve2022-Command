package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorFlip;
import frc.robot.Constants.MotorValue;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed;
import static frc.robot.Constants.Intake.INTAKE_SPIN_MOTOR_ID;
import static frc.robot.utils.motor.MotorUtil.flip;

/**
 * This {@link IntakeSubsystem} class is designed to handle moving the Intake Motor in the accepting
 * or rejecting direction.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor;

    /** Creates a new {@link IntakeSubsystem} */
    public IntakeSubsystem() {
        this.intakeMotor = new CANSparkMax(INTAKE_SPIN_MOTOR_ID, kBrushed);
    }

    /** Spins the Intake Motor in the Accepting Direction. */
    public void spinAccept() {
        intakeMotor.set(flip(MotorValue.SPIN_INTAKE_ACCEPT, MotorFlip.INTAKE_FLIPPED));
    }

    /** Spins the Intake Motor in the Rejecting Direction. */
    public void spinReject() {
        intakeMotor.set(flip(-MotorValue.SPIN_INTAKE_ACCEPT, MotorFlip.INTAKE_FLIPPED));
    }

    /** Stops the Intake Motor from Spinning. */
    public void stop() {
        intakeMotor.set(0);
    }
}