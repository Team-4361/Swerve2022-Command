package frc.robot.utils.power;

/**
 * This is the main breaker of the robot, extended off a {@link PowerBreaker}
 * and has extra features designed for managing the actual battery, rather
 * than a specific fuse.
 */
public class MainPowerBreaker extends PowerBreaker {
    private double totalJoules = 0;

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

    /**
     * Updates the energy readings specific to the {@link MainPowerBreaker}. This includes the total amount of
     * consumed Joules (watt-seconds) as indicated by the Robot.
     *
     * @param voltage The battery voltage reported by the {@link edu.wpi.first.wpilibj.PowerDistribution} system.
     * @param current The total amount of amperes the Robot is consuming.
     * @param consumedEnergy The total amount of energy in <b>Joules</b> the Robot has consumed since reset.
     */
    public void update(double voltage, double current, double consumedEnergy) {
        super.update(voltage, current);
        this.totalJoules = consumedEnergy;
    }

    /** @return The total watt-hours consumed by the <b>entire Robot.</b> */
    public double getWattHours() {
        // The method built in to the PDP reports the total energy in Joules, which is measuring the watts per second
        // of operation. (Watt-Hours = Joules / 3600)
        return totalJoules / 3600.0;
    }

    /** @return The total kilowatt-hours consumed by the <b>entire Robot.</b> */
    public double getKiloWattHours() {
        // Since the watt-hours can already be calculated, simply dividing the result by 1000 can give the proper
        // amount. (KW Hours = Watt-Hours / 1000)
        return getWattHours() / 1000;
    }
}
