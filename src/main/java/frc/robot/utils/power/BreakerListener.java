package frc.robot.utils.power;

/**
 * This interface is designed to listen to different events going
 * on with the breaker, and is intended to be called with a set
 * value from another source.
 */
public interface BreakerListener {
    /**
     * Called when a {@link PowerBreaker} is over the maximum allowed current.
     *
     * @param id The ID of the {@link PowerBreaker}.
     * @param instantCurrent The instant current of the {@link PowerBreaker}.
     * @param maxCurrent The maximum current the {@link PowerBreaker} is supposed to be at.
     */
    void breakerOverCurrent(double id, double instantCurrent, double maxCurrent);

    /**
     * Called when the {@link MainPowerBreaker} battery voltage is below the set threshold for an extended amount of
     * time.
     */
    void robotVoltageLow(double instantVoltage, double setVoltage);

    /**
     * Called when the entire robot is over the maximum allowed current.
     *
     * @param instantCurrent The instant current of the {@link MainPowerBreaker}.
     * @param maxCurrent The maximum current the {@link MainPowerBreaker} is supposed to be at.
     */
    void robotOverCurrent(double instantCurrent, double maxCurrent);
}
