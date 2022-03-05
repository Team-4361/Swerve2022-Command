package frc.robot.robot_utils.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import me.wobblyyyy.pathfinder2.robot.components.AbstractMotor;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * This {@link ProtectedAbstractMotor} extends the {@link AbstractMotor} interface
 * and is used to be able to run a motor with stall protection, while having
 * all the features from being extended. This relies on a periodic method to
 * be called constantly, such as in the periodic/execute methods. This also
 * offers some extra features, like timed running.
 *
 * @see CommandBase#execute()
 * @see SubsystemBase#periodic()
 */
public class ProtectedAbstractMotor extends AbstractMotor {
    private double maxTemperatureCelsius = 35;
    private double maxRunningCurrent = 60;
    private double maxStallCurrent = 120;
    private double minimumStallRPM = 200;
    private double minimumVoltage = 9;

    private boolean stopOnError = true;

    private final CANSparkMax sparkMotor;
    private final RelativeEncoder encoder;
    private final ArrayList<MotorListener> motorErrorListeners;
    private MotorIntention motorIntention;

    private Consumer<MotorError> motorErrorMethod;
    
    /**
     * Create a new {@code AbstractMotor} using a {@link Supplier} and a {@link Consumer}.
     *
     * @param setPower    a {@code Consumer} that accepts a double value. This consumer should perform some set of actions
     *                    that actually sets power to the motor and makes it spin.
     *                    
     * @param getPower    a {@code Supplier} that returns the motor's current power. This method typically calls a provided
     *                    method that queries the power from the physical motor. If such a method is not provided, you
     *                    should go about making
     *                    
     * @param detectMotor the motor that is used for detecting temperature, over-current, etc. This is required
     *                    for the {@link ProtectedAbstractMotor} to function, and failure to use this constructor
     *                    will cause an error.
     */
    public ProtectedAbstractMotor(Consumer<Double> setPower, Supplier<Double> getPower, CANSparkMax detectMotor) {
        super(setPower, getPower);
        this.sparkMotor = detectMotor;
        this.encoder = sparkMotor.getEncoder();
        this.motorErrorListeners = new ArrayList<>();
    }

    /**
     * Create a new {@code AbstractMotor} using a {@link Supplier} and a {@link Consumer}.
     *
     * @param setPower    a {@code Consumer} that accepts a double value. This consumer should perform some set of
     *                    actions that actually sets power to the motor and makes it spin.
     *                    
     * @param getPower    a {@code Supplier} that returns the motor's current power. This method typically calls a
     *                    provided method that queries the power from the physical motor. If such a method is not
     *                    provided, you should go about making it function in some other way. Sorry!
     *                    
     * @param isInverted  if this is true, all {@code setPower} and {@code getPower} operations will multiply any
     *                   
     * @param detectMotor the motor that is used for detecting temperature, over-current, etc. This is required
     *                    for the {@link ProtectedAbstractMotor} to function, and failure to use this constructor
     *                    will cause an error.                  
     */
    public ProtectedAbstractMotor(Consumer<Double> setPower, Supplier<Double> getPower, CANSparkMax detectMotor, 
                                  boolean isInverted) {
        super(setPower, getPower, isInverted);
        this.sparkMotor = detectMotor;
        this.encoder = sparkMotor.getEncoder();
        this.motorErrorListeners = new ArrayList<>();
    }

    /**
     * Create a new {@code AbstractMotor} using a {@link Supplier} and a {@link Consumer}.
     *
     * @param setPower      a {@code Consumer} that accepts a double value. This consumer should perform some set of
     *                      actions that actually sets power to the motor and makes it spin.
     *                      
     * @param getPower      a {@code Supplier} that returns the motor's current power. This method typically calls a
     *                      provided method that queries the power from the physical motor. If such a method is not
     *                      provided, you should go about making it function in some other way. Sorry!
     *                      
     * @param isSetInverted if this is true, all {@code setPower} operations will multiply the inputted power by -1.
     *                      
     * @param isGetInverted if this is true, all {@code getPower} operations
     *                      
     * @param detectMotor   the motor that is used for detecting temperature, over-current, etc. This is required
     *                      for the {@link ProtectedAbstractMotor} to function, and failure to use this constructor
     *                      will cause an error.                    
     */
    public ProtectedAbstractMotor(Consumer<Double> setPower, Supplier<Double> getPower, CANSparkMax detectMotor, 
                                  boolean isSetInverted, boolean isGetInverted) {
        super(setPower, getPower, isSetInverted, isGetInverted);
        this.sparkMotor = detectMotor;
        this.encoder = sparkMotor.getEncoder();
        this.motorErrorListeners = new ArrayList<>();
    }

    /**
     * Create a new {@code AbstractMotor} using a {@link Supplier} and a {@link Consumer}.
     *
     * @param setPower      a {@code Consumer} that accepts a double value. This consumer should perform some set of
     *                      actions that actually sets power to the motor and makes it spin.
     *                      
     * @param getPower      a {@code Supplier} that returns the motor's current power. This method typically calls a
     *                      provided method that queries the power from the physical motor. If such a method is not
     *                      provided, you should go about making it function in some other way. Sorry!
     *                      
     * @param isSetInverted if this is true, all {@code setPower} operations will multiply the inputted power by -1.
     *                      
     * @param isGetInverted if this is true, all {@code getPower} operations will multiply the outputted power by -1.
     *                      
     * @param deadband      the motor's deadband. If a power value is set to a motor such that the absolute value of the
     *                      motor's power is less than this value, the motor's power will simply be set to 0. This is an
     *                      option in case you'd like to try to prevent motor
     *                      
     * @param detectMotor  the motor that is used for detecting temperature, over-current, etc. This is required
     *                     for the {@link ProtectedAbstractMotor} to function, and failure to use this constructor
     *                     will cause an error.
     */
    public ProtectedAbstractMotor(Consumer<Double> setPower, Supplier<Double> getPower, CANSparkMax detectMotor,
                                  boolean isSetInverted, boolean isGetInverted, double deadband) {
        super(setPower, getPower, isSetInverted, isGetInverted, deadband);
        this.sparkMotor = detectMotor;
        this.encoder = sparkMotor.getEncoder();
        this.motorErrorListeners = new ArrayList<>();
    }

    /**
     * Returns the temperature reported from the Motor in the Specified Unit.
     *
     * @param unit The {@link TempUnit} that the motor temperature should be output in.
     * @return The converted {@link TempUnit} of the motor.
     */
    public double getMotorTemperature(TempUnit unit) {
        return Temperature.toUnit(sparkMotor.getMotorTemperature(), TempUnit.CELSIUS, unit);
    }

    /** @return Reported Motor Temperature in {@link TempUnit#CELSIUS}. */
    public double getMotorTemperature() {
        return getMotorTemperature(TempUnit.CELSIUS);
    }

    /** @return Motor Output Current. */
    public double getOutputCurrent() {
        return sparkMotor.getOutputCurrent();
    }

    /** @return The {@link AbstractMotor} (non-protected) instance. */
    public AbstractMotor getAbstractMotor() {
        return this;
    }

    /** @return The {@link CANSparkMax} associated with this {@link ProtectedAbstractMotor}. */
    public CANSparkMax getCANMotor() {
        return this.sparkMotor;
    }


    /** @return The {@link RelativeEncoder} with the Motor. */
    public RelativeEncoder getEncoder() {
        return this.encoder;
    }

    /**
     * Sets the maximum allowed operating temperature of the motor. Anything higher then
     * this WILL cause the motor to stop operation/throw an error.
     *
     * @param temp Maximum Temperature Allowed
     * @param unit The {@link TempUnit} this set temperature is in.
     * @return {@link ProtectedAbstractMotor}
     */
    public ProtectedAbstractMotor setMaxTemperature(double temp, TempUnit unit) {
        this.maxTemperatureCelsius = Temperature.toCelsius(temp, unit);
        return this;
    }

    /**
     * Sets the minimum allowed voltage of the motor. Anything lower than this WILL cause the motor
     * to stop operation/throw an error.
     */
    public ProtectedAbstractMotor setMinVoltage(double voltage) {
        this.minimumVoltage = voltage;
        return this;
    }

    /**
     * Sets the maximum allowed total current of the motor, INCLUDING the startup current. Anything
     * higher than this for more than 2 seconds WILL cause the motor to stop operation/throw an error.
     *
     * @param maxCurrent Maximum Current in Amps Allowed.
     * @return {@link ProtectedAbstractMotor}
     */
    public ProtectedAbstractMotor setMaxTotalCurrent(double maxCurrent) {
        this.maxStallCurrent = maxCurrent;
        return this;
    }

    public ProtectedAbstractMotor setMotorErrorMethod(Consumer<MotorError> method) {
        this.motorErrorMethod = method;
        return this;
    }

    /**
     * Sets the maximum allowed running current of the motor, NOT INCLUDING the startup current. Anything
     * higher than this after the initial startup draw WILL cause the motor to stop operation/throw an error.
     *
     * @param maxCurrent Maximum Current in Amps Allowed.
     * @return {@link ProtectedAbstractMotor}
     */
    public ProtectedAbstractMotor setMaximumRunningCurrent(double maxCurrent) {
        this.maxRunningCurrent = maxCurrent;
        return this;
    }

    /**
     * Sets the minimum RPM that motor is allowed to run at, anything lower than this after the initial current
     * draw WILL cause the motor to stop operation/throw an error.
     * @return {@link ProtectedAbstractMotor}
     */
    public ProtectedAbstractMotor setMinimumRPM(double rpm) {
        this.minimumStallRPM = rpm;
        return this;
    }

    /** @return Max Temperature in specified {@link TempUnit} */
    public double getMaxTemperature(TempUnit unit) {
        return Temperature.toUnit(maxTemperatureCelsius, TempUnit.CELSIUS, unit);
    }

    /** @return Max Running Current */
    public double getMaxRunningCurrent() {
        return this.maxRunningCurrent;
    }

    /** @return Max Stall Current */
    public double getMaxStallCurrent() {
        return this.maxStallCurrent;
    }

    /** @return Minimum Stall RPM */
    public double getMinimumStallRPM() {
        return this.minimumStallRPM;
    }

    /** @return Minimum Motor Voltage */
    public double getMinimumVoltage() {
        return this.minimumVoltage;
    }

    /** Sets if the motor should be stopped when there is an error, or just alert about the issue. */
    public ProtectedAbstractMotor setStopOnError(boolean value) {
        this.stopOnError = value;
        return this;
    }

    /** @return If the motor will stop during an error. */
    public boolean getStopOnError() {
        return this.stopOnError;
    }

    /**
     * Set power to the motor, while starting a {@link MotorIntention} to monitor for stalling.
     *
     * <p>
     * This method doesn't just set power to the motor. It also does a couple of other pretty cool things.
     * <ul>
     *     <li>
     *         Ensure the motor's power value fits between the minimum and
     *         maximum power values. If the motor's power value is less than
     *         the minimum power value, the power will be set to the minimum
     *         power value. If the motor's power is greater than the maximum
     *         power value, the power will be set to the maximum power value.
     *     </li>
     *     <li>
     *         Do some cool stuff related to "lazy mode." Go read the
     *         {@link AbstractMotor} class JavaDoc if you're confused about
     *         what this is.
     *     </li>
     * </ul>
     * </p>
     *
     * @param power the power value to set to the motor.
     */
    @Override
    public void setPower(double power) {
        rawSetPower(power);

        // Start a MotorIntention to run the motor, if one hasn't already been created.
        if (this.motorIntention == null) {
            this.motorIntention = new MotorIntention();
        }
    }

    /**
     * Runs a motor with {@link #setPower(double)}, but adds a specific amount of time,
     * in {@link java.util.concurrent.TimeUnit#SECONDS}.
     *
     * @see #setPower(double)
     */
    public void setPowerTimed(double power, double time) {
        rawSetPower(power);

        if (this.motorIntention == null) {
            this.motorIntention = new MotorIntention(time);
        }
    }

    /**
     * This method is expected to be called on a periodic cycle, meaning this will update the motor's
     * intention if one is running, and make sure nothing is stalling. You really only need to run this
     * if you know the motor is running, but it is still recommend to just always call this method.
     */
    public void periodic() {
        if (motorIntention != null) {
            if (!motorIntention.isRunning()) {
                // The motor is not doing anything, remove the Intention.
                this.motorIntention = null;
            } else {
                // Call the periodic method on the motor and update everything.
                this.motorIntention.periodic();
            }
        }
    }

    public void rawSetPower(double power) {
        super.setPower(power);
    }

    /**
     * Adds an {@link MotorListener} for detecting when there are errors with the motor.
     *
     * @param listener {@link MotorListener} to add.
     * @return {@link ProtectedAbstractMotor}
     */
    public ProtectedAbstractMotor addListener(MotorListener listener) {
        this.motorErrorListeners.add(listener);
        return this;
    }

    class MotorIntention {
        /** The time that the motor should be run for, -1 for infinite. */
        private boolean running, timeStopped = false, runTimerEnabled = false;
        private final Timer motorTimer;
        private final Timer runningTimer;
        private double runTime;

        public MotorIntention() {
            this.motorTimer = new Timer();
            this.runningTimer = new Timer();
            this.running = true;
        }

        public MotorIntention(double timer) {
            this();
            this.runTime = timer;
            this.runTimerEnabled = true;
        }

        public boolean isRunning() {
            return this.running;
        }

        private void end(MotorError error) {
            // If it is configured in the settings to stop the motor, do it after an issue occurs.
            if (stopOnError) {
                rawSetPower(0);
            }

            // Make sure any timers are cancelled if they are running.
            stopTimer();

            // Cancel the running of this class, and disable everything.
            this.running = false;

            if (motorErrorMethod != null) {
                motorErrorMethod.accept(error);
            }

            motorErrorListeners.forEach(li -> li.onMotorError(error));
        }

        private void stop() {
            rawSetPower(0);

            // Make sure any timers are cancelled if they are running.
            stopTimer();

            // Cancel the running of this class, and disable everything.
            this.running = false;
        }

        private void startTimer() {
            if (motorTimer.get() == 0) {
                this.timeStopped = false;
                this.motorTimer.reset();
                this.motorTimer.start();
            }
        }

        private void stopTimer() {
            if (!timeStopped) {
                this.motorTimer.stop();
                this.motorTimer.reset();
                this.timeStopped = true;
            }
        }

        /**
         * The method that should be called periodically, and is used to constantly
         * updated the motor, and listen to stalling.
         */
        public void periodic() {
            if (this.running) {
                // The motor has been manually shut off, stop everything.
                if (getPower() == 0) {
                    stop();
                }

                // The motor timer has exceeded, shut everything down.
                else if (runTimerEnabled && runningTimer.get() >= this.runTime) {
                    stop();
                }

                // If the maximum peak current is greater than what it should be, throw an error.
                double current = sparkMotor.getOutputCurrent();
                double temperature = sparkMotor.getMotorTemperature();
                double voltage = sparkMotor.getBusVoltage();
                double rpm = encoder.getVelocity();
                double stallTime;

                if (current > maxRunningCurrent && current < maxStallCurrent && rpm < minimumStallRPM) {
                    // The current level is getting too high, monitor the levels and if they are high for too long by
                    // starting a Timer, if it has not already been started.
                    stallTime = motorTimer.get();

                    if (stallTime == 0) {
                        startTimer();
                    } else if (stallTime > 1) {
                        end(MotorError.OVER_CURRENT);
                    }

                } else if (current > maxStallCurrent) {
                    // The current is too high, immediately shut everything down.
                    end(MotorError.OVER_CURRENT);
                } else {
                    // We are not being stalled, everything is normal so stop and reset the timers.
                    stopTimer();
                }

                if (temperature > maxTemperatureCelsius) {
                    end(MotorError.OVER_TEMPERATURE);
                }

                if (voltage < minimumVoltage) {
                    end(MotorError.LOW_VOLTAGE);
                }
            }
        }
    }
}