package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot_utils.encoder.ConcurrentRotationalEncoder;
import frc.robot.robot_utils.motor.MotorUtil;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Climber.*;
import static frc.robot.Constants.MotorFlip.CLIMBER_LEFT_FLIPPED;
import static frc.robot.Constants.MotorValue.CLIMBER_SPEED;
import static frc.robot.robot_utils.motor.MotorUtil.getMotorValue;

public class LeftClimberSubsystem extends SubsystemBase {
    private final CANSparkMax climberMotor = new CANSparkMax(L_CLIMBER_ID, kBrushless);
    private final Debouncer debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

    /**
     * This encoder is used for being able to run the climber at a high speed until nearly the max rotations,
     * then start ramping down the speed until the limit switch is hit.
     */
    private final ConcurrentRotationalEncoder encoder;

    private final DigitalInput blSwitch, tlSwitch;

    private boolean isDone = false;

    public void setDone(boolean done) {
        this.isDone = done;
    }

    public boolean getDone() {
        return this.isDone;
    }

    public LeftClimberSubsystem() {
        this.blSwitch = new DigitalInput(BL_LIMIT_ID);
        this.tlSwitch = new DigitalInput(TL_LIMIT_ID);

        this.encoder = new ConcurrentRotationalEncoder(climberMotor)
                .setFlipped(CLIMBER_LEFT_FLIPPED);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("bl switch", blSwitch.get());
        SmartDashboard.putBoolean("tl switch", tlSwitch.get());

        SmartDashboard.putNumber("climber: left encoder", getRotations());
    }

    /** @return If the motor is over 40C, which is a good sign that it's stalling */
    public boolean isDangerousTemperature() {
        return climberMotor.getMotorTemperature() >= 40;
    }

    public void stop() {
        climberMotor.stopMotor();
    }

    public void raise() {
        MotorUtil.runMotor(climberMotor, getMotorValue(-CLIMBER_SPEED, CLIMBER_LEFT_FLIPPED));
    }

    public void lower() {
        MotorUtil.runMotor(climberMotor, getMotorValue(CLIMBER_SPEED, CLIMBER_LEFT_FLIPPED));
    }

    public void zero() {
        this.encoder.reset();
    }

    public double getRotations() {
        return this.encoder.getAbsoluteRotations();
    }

    public boolean isTopLeftSwitchPressed() {
        return debouncer.calculate(tlSwitch.get());
    }

    public boolean isBottomLeftSwitchPressed() {
        return debouncer.calculate(blSwitch.get());
    }
}