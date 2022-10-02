package frc.robot.utils.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.MotorValue;

import java.util.stream.IntStream;

/**
 * Basic Motor Utilities, designed for motors that have an increased risk of
 * something going wrong or stalling, and has detection for it. This also
 * lets you run multiple motors at a time, and have some other purposes
 * as well.
 */
public class MotorUtil {
    /** @return Motor value, reversed if flipped is true. */
    public static double flip(double speed, boolean flipped) {
        return (flipped) ? -speed : speed;
    }

    /** @return If the actual value is within tolerance is the actual value. */
    public static boolean inTolerance(double expected, double actual, double tolerance) {
        return Math.abs(expected - actual) <= tolerance;
    }

    /**
     * Runs a group of motors all at the same speed.
     *
     * @param motors Motor Group
     * @param speed  Constant Speed
     */
    public static void multiSet(CANSparkMax[] motors, double speed) {
        for (CANSparkMax motor : motors)
            motor.set(speed);
    }

    public static double deadzone(double value, double deadzone) {
        return Math.abs(value) > deadzone ? value : 0;
    }

    /**
     * Runs a group of motors at different speeds, based on the order.
     *
     * @param motors Motor Group
     * @param speeds Speed Group
     */
    public static void multiSet(CANSparkMax[] motors, double[] speeds) {
        if (motors.length == speeds.length)
            IntStream.range(0, motors.length).forEach(i -> motors[i].set(speeds[i]));
    }
}
