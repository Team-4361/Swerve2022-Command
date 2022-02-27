package frc.robot.robot_utils.encoder;

import com.revrobotics.RelativeEncoder;

import me.wobblyyyy.pathfinder2.geometry.Angle;
import me.wobblyyyy.pathfinder2.robot.components.Motor;
import me.wobblyyyy.pathfinder2.time.Time;

public class ConcurrentRotationalEncoder {
    private final Motor motor;
    private final RelativeEncoder encoder;

    private double absoluteRotations;
    private double relativeRotations;
    private double velocity;
    private boolean rpmFlipped;
    private double lastTimeMs;

    public ConcurrentRotationalEncoder(Motor motor,
                                       RelativeEncoder encoder) {
        this.motor = motor;
        this.encoder = encoder;
    }

    private static double calculateRotationChange(double encoderRpm,
                                                  double elapsedTimeMs) {
        return encoderRpm / (60 / (elapsedTimeMs / 1_000));
    }

    public void periodic() {
        if (lastTimeMs == 0) lastTimeMs = Time.ms();
        double elapsedTimeMs = Time.ms() - lastTimeMs;
        if (elapsedTimeMs < 200) return;

        velocity = getMotorValue(encoder.getVelocity(), rpmFlipped);
        if (!inTolerance(0, velocity, 2)) {
            relativeRotations =
                getMotorValue(encoder.getPosition(), rpmFlipped);

            relativeRotations =
                calculateRotationChange(velocity, elapsedTimeMs);

            absoluteRotations += relativeRotations;
        }
        lastTimeMs = Time.ms();
    }

    public static boolean inTolerance(double expected, double actual, double tolerance) {
        return Math.abs(expected - actual) <= tolerance;
    }

    public static double getMotorValue(double speed, boolean flipped) {
        return (flipped) ? -speed : speed;
    }

    public ConcurrentRotationalEncoder setFlipped(boolean flipped) {
        rpmFlipped = flipped;

        return this;
    }

    public double getAbsoluteRotations() {
        return absoluteRotations;
    }

    public double getAbsoluteAngle() {
        return absoluteRotations * 360;
    }

    public double getRelativeRotations() {
        return relativeRotations;
    }

    public double getVelocity() {
        return velocity;
    }

    public Angle getAngle() {
        return Angle.fixedDeg(getAbsoluteAngle());
    }

    public ConcurrentRotationalEncoder resetZero() {
        absoluteRotations = 0;
        relativeRotations = 0;
        velocity = 0;

        return this;
    }

    public void nonBlockingSetMotorRotations(double degrees,
                                             double speed) {
        if (getAbsoluteRotations() < degrees)
            motor.setPower(getMotorValue(speed, rpmFlipped));
        else
            motor.setPower(getMotorValue(-speed, rpmFlipped));
    }

    public void nonBlockingSetMotorAngle(double angleInRotations,
                                         double speed) {
        nonBlockingSetMotorRotations(angleInRotations / 360, speed);
    }
}
