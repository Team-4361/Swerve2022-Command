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
    private final SparkMaxPIDController cont;
    private final DigitalInput adjustorLimit;

    private double target = 0;

    public AngleAdjustSubsystem() {
        this.adjustor = new CANSparkMax(ADJUSTOR_MOTOR_ID, kBrushless);
        this.adjustorLimit = new DigitalInput(ADJUSTOR_LIMIT_PORT);

        this.cont = adjustor.getPIDController();
        this.encoder = adjustor.getEncoder();

        cont.setP((double) 1 / 10.5);
        adjustor.enableVoltageCompensation(12);
        adjustor.setInverted(ADJUSTOR_FLIPPED);

        zero();
    }

    @Override
    public void periodic() {
        if (adjustorLimit.get()) {
            // This should prevent the system from increasing the angle too much.
            target = getAngle() - 1;
        }

        cont.setReference(target, ControlType.kPosition, 0);

        SmartDashboard.putNumber("Adjustor Rotations:", getPosition());
        SmartDashboard.putNumber("Adjustor Angle:", getAngle());
    }

    public void setAngle(double target) {
        double targetRotations = target / DEGREES_PER_ROTATION;

        if (targetRotations > MAX_ROTATION) {
            this.target = MAX_ROTATION;
        } else if (targetRotations < 0) {
            this.target = 0;
        } else {
            this.target = target / DEGREES_PER_ROTATION;
        }
    }

    public double getAngle() {
        return getPosition() * DEGREES_PER_ROTATION;
    }

    public void zero() {
        encoder.setPosition(0);
        target = 0;
    }

    public double getPosition() {
        return encoder.getPosition();
    }
}
