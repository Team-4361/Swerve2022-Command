package frc.robot.robot_utils.motor;

public enum MotorError {
    /**
     * Used when the motor has exceeded its maximum stall/running current.
     *
     * @see ProtectedAbstractMotor#setMaximumRunningCurrent(double)
     * @see ProtectedAbstractMotor#setMaxTotalCurrent(double)
     */
    OVER_CURRENT,

    /**
     * Used when the motor has exceeded its maximum temperature.
     *
     * @see ProtectedAbstractMotor#setMaxTemperature(double, TempUnit)
     */
    OVER_TEMPERATURE,

    /**
     * Used when the voltage received by the motor has dropped past the minimum.
     *
     * @see ProtectedAbstractMotor#setMinVoltage(double)
     */
    LOW_VOLTAGE
}
