package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot_utils.MotorUtil;
import frc.robot.robot_utils.encoder.RotationalAbsoluteEncoder;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.IntakeShooter.ADJUSTOR_GEAR_RATIO;
import static frc.robot.Constants.IntakeShooter.SHOOTER_ADJUSTMENT_PORT;
import static frc.robot.Constants.MotorFlip.ADJUSTOR_FLIPPED;
import static frc.robot.Constants.MotorValue.ADJUSTOR_SPEED;

public class AngleAdjustSubsystem extends SubsystemBase {

    private final CANSparkMax adjustorMotor;
    private final RotationalAbsoluteEncoder absoluteEncoder;

    public AngleAdjustSubsystem() {
        adjustorMotor = new CANSparkMax(SHOOTER_ADJUSTMENT_PORT, kBrushless);
        absoluteEncoder = new RotationalAbsoluteEncoder(adjustorMotor)
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

    public void setAngle(double angle) {
        // If the difference between the two is very slight, don't bother
        // doing anything.
        if (!MotorUtil.inTolerance(angle, getAngle(), 2)) {
            if (getAngle() > angle) {
                MotorUtil.runMotor(adjustorMotor, -MotorUtil.getMotorValue(ADJUSTOR_SPEED, ADJUSTOR_FLIPPED));
            } else {
                MotorUtil.runMotor(adjustorMotor, MotorUtil.getMotorValue(ADJUSTOR_SPEED, ADJUSTOR_FLIPPED));
            }

            while (!MotorUtil.inTolerance(angle, getAngle(), 2)) {
                // Important to prevent CPU hanging.
                Thread.onSpinWait();
            }

            MotorUtil.stopMotor(adjustorMotor);
        }
    }

    public void setRotationsFromBase(double position) {
        setAngle(rotationToAngle(position));
    }
}
