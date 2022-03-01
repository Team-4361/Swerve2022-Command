package frc.robot.robot_utils.encoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import me.wobblyyyy.pathfinder2.geometry.Angle;
import me.wobblyyyy.pathfinder2.robot.components.AbstractMotor;
import me.wobblyyyy.pathfinder2.robot.components.Motor;
import me.wobblyyyy.pathfinder2.time.Time;

public class ConcurrentRotationalEncoder {
    private final Motor motor;
    private final RelativeEncoder encoder;

    private double absoluteRotations;
    private double relativeRotations;
    private double calculatedRotations;
    private double velocity;
    private boolean rpmFlipped;
    private long lastTimeMs;

    public ConcurrentRotationalEncoder(Motor motor, RelativeEncoder encoder) {
        this.motor = motor;
        this.encoder = encoder;
    }

    public ConcurrentRotationalEncoder(CANSparkMax motor) {
        this.motor = new AbstractMotor(motor::set, motor::get, false);
        this.encoder = motor.getEncoder();
    }

    private static double calculateRotationChange(double encoderRpm, double elapsedTimeMs) {
        return encoderRpm / (60 / (elapsedTimeMs / 1_000));
    }

    public void periodic() {
        if (lastTimeMs == 0) lastTimeMs = Time.longMs();
        long elapsedTimeMs = Time.longMs() - lastTimeMs;

        if (elapsedTimeMs < 200) return;

        velocity = getMotorValue(encoder.getVelocity(), rpmFlipped);
        if (!inTolerance(0, velocity, 2)) {
            relativeRotations = getMotorValue(encoder.getPosition(), rpmFlipped);
            calculatedRotations = calculateRotationChange(velocity, elapsedTimeMs);
            absoluteRotations += calculatedRotations;
        }
        lastTimeMs = Time.longMs();
    }

    public static boolean inTolerance(double expected, double actual, double tolerance) {
        return Math.abs(expected - actual) <= tolerance;
    }

    public static double getMotorValue(double speed, boolean flipped) {
        return (flipped) ? -speed : speed;
    }

    public ConcurrentRotationalEncoder setFlipped(boolean flipped) {
        rpmFlipped = flipped;
        motor.invert(flipped);

        return this;
    }

    public double getAbsoluteRotations() {return absoluteRotations;}

    public double getAbsoluteAngle() {return absoluteRotations * 360;}

    public double getRelativeRotations() {return relativeRotations;}

    public double getVelocity() {return velocity;}

    public Angle getAngle() {return Angle.fixedDeg(getAbsoluteAngle());}

    public void reset() {
        absoluteRotations = 0;
        relativeRotations = 0;
        calculatedRotations = 0;
        velocity = 0;
    }

    public void setMotorRotations(double degrees, double speed) {
        if (getAbsoluteRotations() < degrees) {
            motor.setPower(speed);
        } else {
            motor.setPower(-speed);
        }
    }

    public void setMotorAngle(double angleInRotations, double speed) {
        setMotorRotations(angleInRotations / 360, speed);
    }
}
