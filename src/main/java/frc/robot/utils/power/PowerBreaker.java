package frc.robot.utils.power;

import edu.wpi.first.math.filter.LinearFilter;

public class PowerBreaker {
    public static final int MAIN_BREAKER_ID = -1;
    private final double currentLimit;

    private int fuseNumber = MAIN_BREAKER_ID;

    private double instantCurrent;
    private double instantVoltage;

    private double maxCurrent = 0;
    private double minCurrent = 0;
    private double wattHours = 0, ampHours = 0;

    private long lastUpdated = System.currentTimeMillis();

    private final LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);

    /**
     * @return The maximum current the fuse is able to <b>supply</b> in amps.
     */
    public double getCurrentLimit() {
        return this.currentLimit;
    }

    /**
     * @return If the {@link #instantCurrent} is over the {@link #currentLimit} limit.
     */
    public boolean isOverCurrentLimit() {
        return filter.calculate(this.instantCurrent) > this.currentLimit;
    }

    /** @return The current wattage of the {@link PowerBreaker} */
    public double getWattage() { return instantVoltage * instantCurrent; }

    /** @return The maximum current the breaker has <b>reached</b> in amps. */
    public double getMaximumCurrent() { return this.maxCurrent; }

    /** @return The minimum current the breaker has <b>reached</b> in amps. */
    public double getMinimumCurrent() { return this.minCurrent; }

    /** @return The total watt-hours of the fuse. */
    public double getWattHours() { return this.wattHours; }

    /** @return The total amp-hours of the fuse. */
    public double getAmpHours() { return this.ampHours; }

    /** @return The maximum wattage of the {@link PowerBreaker} */
    public double getWattageLimit() { return currentLimit * instantVoltage; }

    /** @return The {@link #instantCurrent} compared to the {@link #currentLimit} in percentage form. */
    public double getCurrentPercentage() { return (instantCurrent / currentLimit) * 100; }

    /** @return The instant current being used for the specific fuse. */
    public double getCurrent() { return this.instantCurrent; }

    /** @return The instant voltage of the robot. */
    public double getVoltage() { return this.instantVoltage; }

    /** @return The fuse number associated with this breaker. */
    public int getFuseNumber() { return this.fuseNumber; }

    /** @return The total kilowatts of the {@link PowerBreaker}, not sure why this is necessary, but it's here. */
    public double getKiloWatts() { return this.getWattage() / 1000; }

    /**
     * Creates a new {@link PowerBreaker} with the specified values.
     *
     * @param fuseNumber     The number associated with the fuse.
     * @param instantVoltage The instant voltage of the robot.
     * @param instantCurrent The instant current being used for the specific fuse.
     * @param currentLimit   The maximum current the fuse is able to supply in amps.
     */
    public PowerBreaker(int fuseNumber, double instantVoltage, double instantCurrent, double currentLimit) {
        this.currentLimit = currentLimit;
        this.instantCurrent = instantCurrent;
        this.instantVoltage = instantVoltage;
        this.fuseNumber = fuseNumber;
    }

    /**
     * Creates a new {@link PowerBreaker} with only a fuseNumber, used when
     * being updated continuously.
     *
     * @param fuseNumber   The number associated with the fuse.
     * @param currentLimit The maximum current the fuse is able to supply in amps.
     */
    public PowerBreaker(int fuseNumber, double currentLimit) {
        this.fuseNumber = fuseNumber;
        this.currentLimit = currentLimit;
    }

    /**
     * Creates a new {@link PowerBreaker} with only a currentLimit, designed
     * for the main breaker.
     *
     * @param currentLimit The maximum current the fuse is able to supply in amps.
     */
    public PowerBreaker(double currentLimit) {
        this.currentLimit = currentLimit;
    }

    /**
     * Updates the breaker with the specified voltage and current, running
     * all the calculations automatically. This doubles as a periodic method
     * so it can also be used like that.
     *
     * @param voltage The current voltage.
     * @param current The current amperage.
     */
    public void update(double voltage, double current) {
        this.instantVoltage = voltage;
        this.instantCurrent = current;

        // Run minimum and maximum calculations.
        if (current > maxCurrent) {
            this.maxCurrent = current;
        } else if (current < minCurrent) {
            this.minCurrent = current;
        }

        // Add amp-hours, watt-hours, and kilowatt-hours;
        double hoursUpdatedMillis = (System.currentTimeMillis() - lastUpdated) / 3600000d;

        wattHours += getWattage() * hoursUpdatedMillis;
        ampHours += getCurrent() * hoursUpdatedMillis;

        this.lastUpdated = System.currentTimeMillis();
    }
}
