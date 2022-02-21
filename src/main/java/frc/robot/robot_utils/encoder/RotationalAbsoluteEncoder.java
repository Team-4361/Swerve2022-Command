package frc.robot.robot_utils.encoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import frc.robot.robot_utils.MotorUtil;
import me.wobblyyyy.pathfinder2.robot.components.Motor;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.robot_utils.encoder.RotationalAbsoluteEncoder.*;


public class RotationalAbsoluteEncoder {

    protected static CANSparkMax canMotor;
    protected static RelativeEncoder canEncoder;

    private Thread monitorThread;

    protected static double absoluteRotations = 0, relativeRotations = 0, velocity = 0;
    protected static boolean rpmFlipped = false;

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

    public RotationalAbsoluteEncoder start() {
        monitorThread = new Thread(new MotorMonitor());
        monitorThread.start();
        return this;
    }

    public void stop() {
        monitorThread.interrupt();
    }

    // If the RPM is flipped from what it should be going, enable this.
    public RotationalAbsoluteEncoder setFlipped(boolean flipped) {
        rpmFlipped = flipped;
        return this;
    }

    public boolean isRunning() {
        return monitorThread.isAlive();
    }

    // Returns the rotations as emulated by an Absolute Encoder.
    public double getAbsoluteRotations() {
        return absoluteRotations;
    }

    // Returns the rotations as a regular Relative Encoder.
    public double getRelativeRotations() {
        return relativeRotations;
    }

    // Returns RPM of Motor, adjusted for flipping
    public double getVelocity() {
        return velocity;
    }
}

class MotorMonitor extends Thread {
    // Calculates the rotations that should be increased or decreased from the total, based on RPM
    // and time difference between the two changes.
    private double calculateRotationChange(double rpm, double diffSeconds) {
        return rpm/(60/diffSeconds);
    }

    @Override
    public void run() {
        long lastTime = System.currentTimeMillis();
        double rotations;

        while (!isInterrupted()) {
            // Check every 200 milliseconds to avoid bouncing and false values.
            if (System.currentTimeMillis() > lastTime +200) {
                // Get the motor RPM value based on if it's flipped or not.
                velocity = MotorUtil.getMotorValue(canEncoder.getVelocity(), rpmFlipped);

                // Prevents bouncing errors by ignoring the value if it's very close to zero.
                if (!MotorUtil.inTolerance(0, velocity, 2)) {
                    relativeRotations = MotorUtil.getMotorValue(canEncoder.getPosition(), rpmFlipped);

                    // Change the RPM to the actual amount of rotations based on the time it been.
                    rotations = calculateRotationChange(velocity, 0.2);

                    // Add or subtract to the total rotations based on the value being + or -
                    absoluteRotations += rotations > 0 ? rotations : -rotations;
                }

                // Finally, reset the lastTime, so it constantly updates every 200ms.
                lastTime = System.currentTimeMillis();
            }
        }
    }
}
