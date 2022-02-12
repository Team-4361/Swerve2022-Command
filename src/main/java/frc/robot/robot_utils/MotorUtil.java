package frc.robot.robot_utils;

import java.util.concurrent.TimeUnit;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MotorValue;

public class MotorUtil {
	public static CANSparkMax stalledMotor;


    /** @return Motor value, reversed if flipped is true. */
    public static double getMotorValue(double speed, boolean flipped) {
		return (flipped) ? -speed : speed;
    }
    
    public static void runMotorTimed(CANSparkMax motor, double speed, int delayTime) {
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    long stopTime = System.currentTimeMillis() + delayTime;
                    motor.set(speed);

                    // Add a delay before checking for stall current to reduce the possibility of the
                    // extra power and low speed of starting the motor to activate the protection.
                    TimeUnit.MILLISECONDS.sleep(100);

                    while (System.currentTimeMillis() <= stopTime) {
                        if (!isStalled(motor)) {
                            motor.set(speed);
                        } else {
                            motor.set(0);
                            SmartDashboard.putBoolean("jammed", true);
                        }
                    }
                    motor.set(0);
                } catch (InterruptedException e) {}
                Thread.currentThread().interrupt();
            }
        }).start();
    }

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

	public static CANSparkMax getStalledMotor() {
		CANSparkMax motor = stalledMotor;
		stalledMotor = null;

		return motor;
	}

	public static void runMotor(CANSparkMax motor, double speed) {
		new Thread(new Runnable() {
			@Override
			public void run() {
				try {
					// While the motor is not stopped, continously check if the motor is
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
				} catch (InterruptedException e) {}
			}
		}).start();
	}
}
