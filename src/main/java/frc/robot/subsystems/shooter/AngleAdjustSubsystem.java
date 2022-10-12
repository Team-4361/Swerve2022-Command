package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.revrobotics.CANSparkMax.ControlType.kPosition;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.MotorFlip.ADJUSTOR_FLIPPED;
import static frc.robot.Constants.ShooterAdjustor.*;

/**
 * This {@link AngleAdjustSubsystem} is designed to allow the Robot's Shooter Adjustor to align with a specific
 * target angle, thus making the shooting process more precise by allowing the pitch to be changed. By default,
 * it will automatically be able to a target angle, however if desired it can be put into a teleoperated mode.
 */
public class AngleAdjustSubsystem extends SubsystemBase {
    private final CANSparkMax adjustor;
    private final RelativeEncoder encoder;
    private final DigitalInput limitSwitch;
    private final SparkMaxPIDController pidController;

    private double targetRotations = 0.0;
    private boolean teleopMode = false;

    public AngleAdjustSubsystem() {
        this.adjustor = new CANSparkMax(ADJUSTOR_MOTOR_ID, kBrushless);
        this.limitSwitch = new DigitalInput(ADJUSTOR_LIMIT_PORT);

        this.pidController = adjustor.getPIDController();
        this.encoder = adjustor.getEncoder();

        adjustor.enableVoltageCompensation(12);
        adjustor.setInverted(ADJUSTOR_FLIPPED);
        pidController.setP((double)1/3.5);

        zero();
    }

    @Override
    public void periodic() {
        if (!teleopMode)
            pidController.setReference(targetRotations, kPosition, 0);

        SmartDashboard.putNumber("Adjustor Rotations:", getPosition());
        SmartDashboard.putNumber("Adjustor Angle:", getAngle());
    }

    public void setAngle(double target) {
        double targetRotations = target/DEGREES_PER_ROTATION;

        if(targetRotations > MAX_ROTATION){
            this.targetRotations = MAX_ROTATION;
        } else if(targetRotations < 0){
            this.targetRotations = 0;
        }
        else {
            this.targetRotations = target/DEGREES_PER_ROTATION;
        }
    }


    public void increaseAdjustMotor(double speed) {
        translateAdjustor(speed);
    }

    public void decreaseAdjustMotor(double speed) {
        translateAdjustor(-speed);
    }

    public void stopAdjustMotor() {
        translateAdjustor(0);
    }

    /**
     * A <b>POSITIVE</b> number should raise the adjustor, and a <b>NEGATIVE</b> number should lower the adjustor.
     * <p></p>
     * Teleop mode is automatically enabled if speed value is not equal to zero, and disabled when is zero.
     * 
     * @param speed The speed to translate.
     */
    public void translateAdjustor(double speed) {
        if (speed == 0 && teleopMode) {
            // Set the target angle to the current rotations to freeze the value and prevent the PIDController from
            // automatically adjusting to the previous value.
            targetRotations = encoder.getPosition();
            teleopMode = false;
        }
        if (speed != 0 && !teleopMode)
            teleopMode = true;

        adjustor.set(speed);
    }

    public double getAngle() {
        return getPosition() * DEGREES_PER_ROTATION;
    }

    public void zero() {
        encoder.setPosition(0);
    }

    public double getPosition() {
        return encoder.getPosition();
    }
}
