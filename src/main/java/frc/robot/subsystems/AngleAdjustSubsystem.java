package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot_utils.MotorUtil;
import frc.robot.robot_utils.encoder.RotationalAbsoluteEncoder;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.MotorFlip.ADJUSTOR_FLIPPED;
import static frc.robot.Constants.MotorValue.ADJUSTOR_SPEED;
import static frc.robot.Constants.ShooterAdjustor.ADJUSTOR_GEAR_RATIO;
import static frc.robot.Constants.ShooterAdjustor.ADJUSTOR_MOTOR_ID;

public class AngleAdjustSubsystem extends SubsystemBase {

    private final RotationalAbsoluteEncoder absoluteEncoder;
    private final CANSparkMax adjustMotor;

    public AngleAdjustSubsystem() {
        adjustMotor = new CANSparkMax(ADJUSTOR_MOTOR_ID, kBrushless);

        absoluteEncoder = new RotationalAbsoluteEncoder(adjustMotor)
                .setFlipped(ADJUSTOR_FLIPPED)
                .start();
    }

    public double rotationToAngle(double rotation) {
        return ((Math.abs(rotation) * 360) * (1 / ADJUSTOR_GEAR_RATIO));
    }

    public double getAngle() {
        return rotationToAngle(Math.abs(absoluteEncoder.getAbsoluteRotations()));
    }

    public boolean atDesiredAngle(double desired, int tolerance) {
        return (MotorUtil.inTolerance(desired, getAngle(), tolerance));
    }

    private double angleToRotation(double angle) {
        return ((ADJUSTOR_GEAR_RATIO * angle) / 360);
    }

    public void setAngle(double angle) {
        absoluteEncoder.setMotorRotations(angleToRotation(angle), ADJUSTOR_SPEED);
    }

    public void setRotationsFromBase(double position) {
        setAngle(rotationToAngle(position));
    }

    public CANSparkMax getAdjustorMotor() {
        return adjustMotor;
    }
}
