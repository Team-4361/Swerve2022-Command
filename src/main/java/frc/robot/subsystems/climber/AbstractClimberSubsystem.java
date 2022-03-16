package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorValue;
import frc.robot.robot_utils.encoder.ConcurrentRotationalEncoder;

public class AbstractClimberSubsystem extends SubsystemBase {
    protected final DigitalInput bottomLimit;
    protected final DigitalInput topLimit;
    private final CANSparkMax climberMotor;
    private final ConcurrentRotationalEncoder encoder;
    private boolean isDone = false;

    public AbstractClimberSubsystem(DigitalInput bottomLimit,
                                    DigitalInput topLimit,
                                    CANSparkMax climberMotor,
                                    boolean isFlipped) {
        this.bottomLimit = bottomLimit;
        this.topLimit = topLimit;
        this.climberMotor = climberMotor;
        this.encoder = new ConcurrentRotationalEncoder(climberMotor)
                .setFlipped(isFlipped);
    }

    public void setDone(boolean done) {
        this.isDone = done;
    }

    public boolean getDone() {
        return this.isDone;
    }

    public boolean isDangerousTemperature() {
        return climberMotor.getMotorTemperature() >= 40;
    }

    public void stop() {
        climberMotor.stopMotor();
    }

    /**
     * translate the single side of the climber subsystem.
     *
     * @param power the power value to set to the climber. a positive power
     *              value will lower the climber, while a negative power
     *              value will raise the climber.
     */
    public void translateClimber(double power) {
        // power > 0: climber is being LOWERED
        // power < 0: climber is being RAISED
        
        if (power > 0 && bottomLimit.get()) power = 0;
        else if (power < 0 && topLimit.get()) power = 0;

        climberMotor.set(power);
    }

    public void raise() {
        translateClimber(-MotorValue.CLIMBER_SPEED);
    }

    public void lower() {
        translateClimber(MotorValue.CLIMBER_SPEED);
    }

    public void zero() {
        this.encoder.reset();
    }

    public double getRotations() {
        return this.encoder.getAbsoluteRotations();
    }
}
