package frc.robot.robot_utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MotorValue;

import java.util.concurrent.TimeUnit;

/**
 * Basic Motor Utilities, designed for motors that have an increased risk of
 * something going wrong or stalling, and has detection for it. This also
 * lets you run multiple motors at a time, and have some other purposes
 * as well.
 */
public class MotorUtil {

    /**
     * The motor that is currently being stalled.
     */
    private static CANSparkMax stalledMotor;

    /**
     * @return Motor value, reversed if flipped is true.
     */
    public static double getMotorValue(double speed, boolean flipped) {
        return (flipped) ? -speed : speed;
    }

    /**
     * @return If the actual value is within tolerance is the actual value
     */
    public static boolean inTolerance(double expected, double actual, double tolerance) {
        return Math.abs(expected - actual) <= tolerance;
    }

    /**
     * Runs a Motor at a specified speed on a timer with Stall Protection.
     *
     * @param motor     Motor to Run
     * @param speed     Speed to Run Motor At
     * @param delayTime Time in Milliseconds to Hold Motor For
     */
    public static void runMotorTimed(CANSparkMax motor, double speed, int delayTime) {
        new Thread(() -> {
            try {
                long stopTime = System.currentTimeMillis() + delayTime;
                motor.set(speed);

                // Add a delay before checking for stall current to reduce the possibility of the
                // extra power and low speed of starting the motor to activate the protection.
                TimeUnit.MILLISECONDS.sleep(100);

                while (System.currentTimeMillis() <= stopTime) {
                    motor.set(!isStalled(motor) ? speed : 0);
                }

                motor.set(0);
            } catch (InterruptedException ignored) {}
            Thread.currentThread().interrupt();
        }).start();
    }

    /**
     * If the motors is under the Constant Specified RPM, and above Maximum Allowed Output,
     * then return True. Otherwise, return False.
     *
     * @param motor Motor to Test
     * @return If motor is stalled
     */
    public static boolean isStalled(CANSparkMax motor) {
        if (MotorValue.CURRENT_MEASURING) {
            RelativeEncoder encoder = motor.getEncoder();
            double velocity = encoder.getVelocity();

            if (Math.abs(velocity) < MotorValue.STALL_RPM && motor.getOutputCurrent() > MotorValue.STALL_CURRENT) {
                // The motor has been stalled, return true.

                SmartDashboard.putBoolean("MotorUtil: Motor Stalled", true);
                SmartDashboard.putNumber("MotorUtil: Stalled #ID", motor.getDeviceId());

                stalledMotor = motor;
                return true;
            } else {
                // The motor is not currently stalled, return false.
                return false;
            }
        } else {
            // Current Measuring is disabled, always return false.
            return false;
        }
    }

    /**
     * @return Stalled Motor, and resets.
     */
    public static CANSparkMax getStalledMotor() {
        CANSparkMax motor = stalledMotor;
        stalledMotor = null;

        return motor;
    }

    /**
     * Runs a motor at a specified speed with Stall Protection
     *
     * @param motor Running Motor
     * @param speed Speed to Run Motor
     */
    public static void runMotor(CANSparkMax motor, double speed) {
        /*
        new Thread(() -> {
            try {
                // While the motor is not stopped, continuously check if the motor is
                // being stalled by anything, and if so then stop the motor and put it
                // in the detection.
                motor.set(speed);

                // Add a delay before checking for stall current to reduce the possibility of the
                // extra power and low speed of starting the motor to activate the protection.
                TimeUnit.MILLISECONDS.sleep(100);

                while (motor.get() != 0) {
                    if (isStalled(motor)) {
                        // The motor is stalled, turn off the motor and cancel everything.
                        motor.set(0);
                    } else {
                        motor.set(speed);
                    }
                }
                Thread.currentThread().interrupt();
            } catch (InterruptedException ignored) {
            }
        }).start();
        */
        motor.set(speed);
    }

    /**
     * Runs a group of motors all at the same speed.
     *
     * @param motors Motor Group
     * @param speed  Constant Speed
     */
    public static void runMotors(CANSparkMax[] motors, double speed) {
        for (CANSparkMax motor : motors) {
            runMotor(motor, speed);
        }
    }

    /**
     * Runs a group of motors at different speeds, based on the order.
     *
     * @param motors Motor Group
     * @param speeds Speed Group
     */
    public static void runMotors(CANSparkMax[] motors, double[] speeds) {
        for (int i = 0; i < motors.length - 1; i++) {
            runMotor(motors[i], speeds[i]);
        }
    }

    /**
     * Stops a group of motors at the same time.
     *
     * @param motors Motor Group
     */
    public static void stopMotors(CANSparkMax[] motors) {
        for (CANSparkMax motor : motors) {
            motor.set(0);
        }
    }

    /**
     * Stops a specified motor, setting it to zero.
     *
     * @param motor The motor that should be stopped.
     */
    public static void stopMotor(CANSparkMax motor) {
        motor.set(0);
    }

    /**
     * Returns whether any of the motors specified in the group have been stalled.
     *
     * @param motors Motor Group to Check
     * @return If any of the motors in the group have been stalled.
     */
    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    public static boolean isAnyStalled(CANSparkMax[] motors) {
        for (CANSparkMax motor : motors) {
            if (isStalled(motor)) {
                return true;
            }
        }
        return false;
    }
}
