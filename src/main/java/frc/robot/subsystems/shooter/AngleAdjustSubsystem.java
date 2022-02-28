package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot_utils.MotorUtil;
import frc.robot.robot_utils.encoder.RotationalAbsoluteEncoder;
import me.wobblyyyy.pathfinder2.geometry.Angle;
import me.wobblyyyy.pathfinder2.revrobotics.SparkMaxMotor;

import static frc.robot.Constants.MotorFlip.ADJUSTOR_FLIPPED;
import static frc.robot.Constants.ShooterAdjustor.ADJUSTOR_GEAR_RATIO;
import static frc.robot.Constants.ShooterAdjustor.ADJUSTOR_MOTOR_ID;

public class AngleAdjustSubsystem extends SubsystemBase {
    private final SparkMaxMotor adjustor;
    private final RotationalAbsoluteEncoder absoluteEncoder;
    private final PIDController controller;
    private Angle targetAngle;

    public AngleAdjustSubsystem() {
        adjustor = SparkMaxMotor.brushless(ADJUSTOR_MOTOR_ID);
        absoluteEncoder = new RotationalAbsoluteEncoder(adjustor.getSpark())
                .setFlipped(ADJUSTOR_FLIPPED);
        controller = new PIDController(1 / 90, 0, 0);
        controller.setSetpoint(0.0);
    }

    public double rotationToAngle(double rotation) {
        return ((Math.abs(rotation) * 360) * (1 / ADJUSTOR_GEAR_RATIO));
    }

    public double getAngle() {
        return rotationToAngle(Math.abs(
                    absoluteEncoder.getAbsoluteRotations()));
    }

    public boolean atDesiredAngle(double desired,
                                  double tolerance) {
        return (MotorUtil.inTolerance(desired, getAngle(), tolerance));
    }

    public void setAngle(double angle) {
        targetAngle = Angle.fixedDeg(angle);
    }

    public void setRotationsFromBase(double position) {
        setAngle(rotationToAngle(position));
    }

    public CANSparkMax getAdjustorMotor() {
        return adjustor.getSpark();
    }

    @Override
    public void periodic() {
        // absoluteEncoder.update();
        // Angle currentAngle = Angle.fixedDeg(getAngle());
        // double delta = Angle.minimumDelta(currentAngle, targetAngle);
        // double adjustorMotorPower = controller.calculate(delta);
        // adjustor.setPower(adjustorMotorPower);
    }
}
