package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.*;
import static frc.robot.Constants.MotorFlip.INTAKE_EXTENDER_FLIPPED;
import static frc.robot.Constants.MotorValue.ACCEPT_SPEED;
import static frc.robot.utils.motor.MotorUtil.flip;
import static frc.robot.utils.motor.MotorUtil.multiSet;

/**
 * This {@link IntakeExtendSubsystem} is designed to handle extending and retracting the 
 * Intake mechanism out of the robot, and is paired with Magnet sensors used to detect when the
 * system cannot be moved any further.
 */
public class IntakeExtendSubsystem extends SubsystemBase {
    private final CANSparkMax[] extendMotors;
    private final DigitalInput blSensor, brSensor, flSensor, frSensor;

    /** Creates a new {@link IntakeExtendSubsystem} */
    public IntakeExtendSubsystem() {
        this.extendMotors = new CANSparkMax[]{
                new CANSparkMax(Intake.L_INTAKE_EXTEND_ID, kBrushless),
                new CANSparkMax(Intake.R_INTAKE_EXTEND_ID, kBrushless)
        };

        this.flSensor = new DigitalInput(Intake.FL_MAGNET_ID);
        this.frSensor = new DigitalInput(Intake.FR_MAGNET_ID);
        this.blSensor = new DigitalInput(Intake.BL_MAGNET_ID);
        this.brSensor = new DigitalInput(Intake.BR_MAGNET_ID);
    }

    /** Extends the Intake with {@link MotorValue#ACCEPT_SPEED} */
    public void extendIntake() {
        multiSet(extendMotors, flip(ACCEPT_SPEED, INTAKE_EXTENDER_FLIPPED));
    }

    /** Retracts the Intake with {@link MotorValue#ACCEPT_SPEED} */
    public void retractIntake() {
        multiSet(extendMotors, flip(-ACCEPT_SPEED, INTAKE_EXTENDER_FLIPPED));
    }

    /**
     * Retracts the Intake with a specified speed.
     * @param speed The speed from 0.0 to 1.0 to retract it with.
     */
    public void retractIntake(double speed) {
        assert speed >= 0;
        multiSet(extendMotors, flip(-speed, INTAKE_EXTENDER_FLIPPED));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intake extended", isExtended());
        SmartDashboard.putBoolean("intake retracted", isRetracted());
    }

    /** @return If the intake is fully extended. */
    public boolean isExtended() {
        return flSensor.get() && frSensor.get();
    }

    /** @return If the intake is fully retracted. */
    public boolean isRetracted() {
        return blSensor.get() && brSensor.get();
    }

    /** Stops the intake extenders from working. */
    public void stop() {
        multiSet(extendMotors, 0);
    }
}
