package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.MotorFlip.ADJUSTOR_FLIPPED;
import static frc.robot.Constants.ShooterAdjustor.*;

public class AngleAdjustSubsystem extends SubsystemBase {
    private final CANSparkMax adjustor;
    private final RelativeEncoder encoder;

    public AngleAdjustSubsystem() {
        this.adjustor = new CANSparkMax(ADJUSTOR_MOTOR_ID, kBrushless);
        this.encoder = adjustor.getEncoder();

        adjustor.enableVoltageCompensation(12);
        adjustor.setInverted(ADJUSTOR_FLIPPED);

        zero();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Adjustor Rotations:", getPosition());
        SmartDashboard.putNumber("Adjustor Angle:", getAngle());
    }

    public void raiseAngle(double speed) {
        translateAdjustor(speed);
    }

    public void lowerAngle(double speed) {
        translateAdjustor(-speed);
    }

    public void stop() {
        translateAdjustor(0);
    }

    /**
     * A POSITIVE number should raise the adjustor, and a NEGATIVE
     * number should lower the adjustor.
     * 
     * @param speed The speed to translate.
     */
    public void translateAdjustor(double speed) {
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
