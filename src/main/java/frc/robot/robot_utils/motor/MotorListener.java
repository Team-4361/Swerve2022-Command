package frc.robot.robot_utils.motor;

/**
 * This listener is designed to be used from {@link ProtectedAbstractMotor}, and is used when then
 * are errors while running the motor such as {@link MotorError#OVER_TEMPERATURE}, {@link MotorError#OVER_CURRENT},
 * or {@link MotorError#LOW_VOLTAGE}.
 */
public interface MotorListener {
    /**
     * This is used when the motor has received a general error, and uses {@link MotorError} to tell what happened.
     *
     * @param error The {@link MotorError} that is thrown when there's an issue with the motor.
     * @see MotorListener
     */
    void onMotorError(MotorError error);
}
