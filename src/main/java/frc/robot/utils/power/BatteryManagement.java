package frc.robot.utils.power;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.temp.TempUnit;
import frc.robot.utils.temp.Temperature;

import java.util.ArrayList;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;

public class BatteryManagement extends MainPowerBreaker {
    private final Map<Integer, Integer> breakerLimits;
    private final PowerDistribution pds;
    private final PowerBreaker[] breakerInstances;

    private final double currentLimit;

    /**
     * Creates a new {@link BatteryManagement} instance, with preset Breaker Limits, and the main breaker. current
     * limit.
     *
     * @param breakerLimits A {@link Map} containing the Breaker ID, and the Breaker Limit in Amps for each entry.
     * @param currentLimit  The current limit of the Main Power Breaker.
     */
    public BatteryManagement(Map<Integer, Integer> breakerLimits, double currentLimit) {
        super(currentLimit);

        this.currentLimit = currentLimit;
        this.breakerLimits = breakerLimits;
        this.breakerInstances = new PowerBreaker[breakerLimits.size()];
        this.pds = new PowerDistribution();

        AtomicInteger counter = new AtomicInteger(0);
        breakerLimits.forEach((id, max) -> {
            this.breakerInstances[counter.get()] = new PowerBreaker(id, max);
            counter.getAndIncrement();
        });
    }

    /** @return A list of all the registered {@link PowerBreaker}'s. */
    public PowerBreaker[] getBreakers() {
        return this.breakerInstances;
    }

    /** @return The specific {@link PowerBreaker} with an ID, if not found then null. */
    public PowerBreaker getBreaker(int id) {
        for (PowerBreaker breaker : this.breakerInstances) {
            if (breaker.getFuseNumber() == id) return breaker;
        }
        return null;
    }

    /** @return The maximum current the {@link MainPowerBreaker} is allowed to have. */
    public double getMainBreakerLimit() {
        return this.currentLimit;
    }

    public Map<Integer, Integer> getBreakerLimits() {
        return this.breakerLimits;
    }

    /**
     * Resets the Total Amount of Energy indicated by the Robot. This will reset all methods related to watt-hours,
     * amp-hours, etc.
     *
     * @return The {@link PowerBreaker} instance.
     */
    public PowerBreaker resetTotalEnergy() {
        pds.resetTotalEnergy();
        return this;
    }

    /**
     * Updates all the {@link PowerBreaker}'s, and is designed to be called every periodic or execution cycle.
     *
     * @see CommandBase#execute()
     * @see SubsystemBase#periodic()
     */
    public void periodic() {
        // Make sure to update the MainBreaker that we are extending off.
        double pdsVoltage = pds.getVoltage();

        // Update the inherited MainPowerBreaker with the total combined Robot Current and the consumed energy.
        this.update(pdsVoltage, pds.getTotalCurrent(), pds.getTotalEnergy());

        for (PowerBreaker breaker: this.breakerInstances) {
            breaker.update(pdsVoltage, pds.getCurrent(breaker.getFuseNumber()));
        }
    }
}
