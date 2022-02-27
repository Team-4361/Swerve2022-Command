package frc.robot.robot_utils.encoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import static frc.robot.robot_utils.encoder.RotationalAbsoluteEncoder.*;

@SuppressWarnings("BooleanMethodIsAlwaysInverted")
public class RotationalAbsoluteEncoder {

    protected static CANSparkMax canMotor;
    protected static RelativeEncoder canEncoder;
    protected static double accuracyFactor = 15;

    /** absolute rotations */
    protected static double absoluteRotations = 0;

    /** relative rotations */
    protected static double relativeRotations = 0;

    /** calculated rotations */
    protected static double calculatedRotations = 0;

    /** raw motor velocity */
    protected static double velocity = 0;

    /** if the RPM is flipped */
    protected static boolean rpmFlipped = false;

    private long lastUpdated, timeDifference;

    // After doing research, experimenting, and looking at my recorded graphs, I discovered
    // that to accurately record the total absolute rotational information, we should be using
    // the RPM of the Encoder, rather than constantly relying on the total rotations of the motor
    // because it is much more accurate that way due to the sensors. The delay should be around
    // 100 milliseconds to prevent bouncing.

    // There were multiple ways I originally wanted to do this for adjusting angles, and I think
    // this is the best one because of the pros/cons of each experiment. These are what I tried:

    // 100% Relative Position Based:
    // - Pros: Very simple, accurate when working
    //
    // - Cons: Often likes to freeze values during testing, looses accuracy quickly, cannot reset
    // position without completely restarting robot or using calibrator, hard to tell where 0 really is,
    // cannot tell exact angle physically (due to relative), only can go up/down adjustments.

    // RPM Based Simulated Absolute (currently using)
    // - Pros: Much more consistent readings due to RPM not freezing, offers protection from accidental
    // moving during operation
    //
    // - Cons: Much more complex/can break, requires constant monitoring, RPM may not be extremely accurate
    // but will be more consistent in general.

    // A hybrid approach (relative for track, rpm for protection) would not be much better,
    // because it will have the same issues as Relative position based, while not having any real advantages.

    public RotationalAbsoluteEncoder(CANSparkMax motor) {
        canMotor = motor;
        canEncoder = motor.getEncoder();
    }

    public static boolean inTolerance(double expected, double actual, double tolerance) {
        return Math.abs(expected - actual) <= tolerance;
    }

    public static double getMotorValue(double speed, boolean flipped) {
        return (flipped) ? -speed : speed;
    }

    // Used when telling the motor to go to a specified position, this is the factor
    // based on the motor speed of tolerance it has.
    //
    // formula is (motor speed*accuracy factor) = tolerance rotations
    public RotationalAbsoluteEncoder setAccuracyFactor(double factor) {
        accuracyFactor = factor;
        return this;
    }

    // If the RPM is flipped from what it should be going, enable this.
    public RotationalAbsoluteEncoder setFlipped(boolean flipped) {
        rpmFlipped = flipped;
        return this;
    }

    // Returns the rotations as emulated by an Absolute Encoder.
    public double getAbsoluteRotations() {
        return absoluteRotations;
    }

    public double getAbsoluteAngle() {
        return absoluteRotations * 360;
    }

    // Returns the rotations as a regular Relative Encoder.
    public double getRelativeRotations() {
        return relativeRotations;
    }

    // Returns RPM of Motor, adjusted for flipping
    public double getVelocity() {
        return velocity;
    }

    // Resets everything to zero for calibration.
    public RotationalAbsoluteEncoder resetZero() {
        absoluteRotations = 0;
        relativeRotations = 0;
        velocity = 0;

        return this;
    }


    private double calculateRotationChange(double rpm, double diffSeconds) {
        return rpm / (60 / diffSeconds);
    }

    /**
     * This is expected to be automatically updated on a scheduled period of time,
     * such as being called from the periodic/execute looped methods on a Subsystem
     * or Command. This is fully non-blocking and doesn't rely on any additional Threads.
     */
    public void update() {
        timeDifference = System.currentTimeMillis() - lastUpdated;

        velocity = getMotorValue(canEncoder.getVelocity(), rpmFlipped);

        if (!inTolerance(0, velocity, 1)) {
            relativeRotations = getMotorValue(canEncoder.getPosition(), rpmFlipped);

            // Change the RPM to the actual amount of rotations based on the time it been.
            calculatedRotations = calculateRotationChange(velocity, (double) timeDifference/1000);

            // Add or subtract to the total rotations based on the value being + or -
            absoluteRotations += calculatedRotations;

            lastUpdated = System.currentTimeMillis();
        }

    }

    public void setMotorRotations(double degrees, double speed) {
        if (getAbsoluteRotations() < degrees) {
            // rotate up
            canMotor.set(getMotorValue(speed, rpmFlipped));
        } else {
            // rotate down
            canMotor.set(getMotorValue(-speed, rpmFlipped));
        }

        // TODO: there is probably a better solution
        while (!inTolerance(degrees, getAbsoluteRotations(), 20)) {
            Thread.onSpinWait();
        }

        canMotor.set(0);
    }

    public void nonBlockingSetMotorRotations(double degrees,
                                             double speed) {
        if (getAbsoluteRotations() < degrees)
            canMotor.set(getMotorValue(speed, rpmFlipped));
        else
            canMotor.set(getMotorValue(-speed, rpmFlipped));
    }

    public void setMotorAngle(double angle, double speed) {
        setMotorRotations(angle / 360, speed);
    }

    public void nonBlockingSetMotorAngle(double angleInRotations,
                                         double speed) {
        nonBlockingSetMotorRotations(angleInRotations / 360, speed);
    }

    public void increaseRotations(double increase, double speed) {
        setMotorRotations(getAbsoluteRotations() + increase, speed);
    }

    public void decreaseRotations(double decrease, double speed) {
        setMotorRotations(getAbsoluteRotations() - decrease, speed);
    }

    public void increaseMotorAngle(double increase, double speed) {
        setMotorAngle(getAbsoluteAngle() + increase, speed);
    }

    public void decreaseMotorAngle(double decrease, double speed) {
        setMotorAngle(getAbsoluteAngle() - decrease, speed);
    }
}
