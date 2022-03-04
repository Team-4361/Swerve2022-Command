package frc.robot.robot_utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import me.wobblyyyy.pathfinder2.robot.components.AbstractMotor;

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
public abstract class ProtectedAbstractMotor extends AbstractMotor {

    /**
     * The maximum temperature in celsius that the motor is allowed to be running at.
      */
    private double maximumTemperature = 35;

    /** The maximum current in amps that the motor is allowed to be running at. */
    private double maximumCurrent = 80; /*A*/
}


class MotorIntention {
    /** The time that the motor should be run for, -1 for infinite. */
    private double motorRuntime = -1;
    private boolean running = false;
    private long startMillis, endMillis;

    public MotorIntention() {}

    public MotorIntention(double runtime) {
        this.motorRuntime = runtime;
    }

    /**
     * The method that should be called periodically, and is used to constantly
     * updated the motor, and listen to stalling.
     */
    public void periodic() {

    }
}
