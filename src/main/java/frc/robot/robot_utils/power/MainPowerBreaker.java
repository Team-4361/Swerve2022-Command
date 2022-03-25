package frc.robot.robot_utils.power;

/**
 * This is the main breaker of the robot, extended off a {@link PowerBreaker}
 * and has extra features designed for managing the actual battery, rather
 * than a specific fuse.
 */
public class MainPowerBreaker extends PowerBreaker {
    /**
     * Creates a new {@link PowerBreaker} with only a currentLimit, designed
     * for the main breaker.
     *
     * @param currentLimit The maximum current the fuse is able to supply in amps.
     */
    public MainPowerBreaker(double currentLimit) {
        super(currentLimit);
    }

    /** @return The <i>estimated</i> battery percentage, 12.4V considered 100%. */
    public double getBatteryPercentage() {
        return Math.min((getVoltage() / 12.4) * 100, 100);
    }
}
