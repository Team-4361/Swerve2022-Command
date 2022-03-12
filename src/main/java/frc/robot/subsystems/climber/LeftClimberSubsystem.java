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
        SmartDashboard.putBoolean("Climber: BL Switch", blSwitch.get());
        SmartDashboard.putBoolean("Climber: TL Switch", tlSwitch.get());

        SmartDashboard.putNumber("Climber: Left Rotations", getRotations());
        SmartDashboard.putNumber("Climber: Left Motor Temp", climberMotor.getMotorTemperature());
    }

    /** @return If the motor is over 40C, which is a good sign that it's stalling */
    public boolean isDangerousTemperature() {
        return climberMotor.getMotorTemperature() >= 40;
    }

    public void stop() {
        climberMotor.stopMotor();
    }

    public void raise() {
        //if (!isDangerousTemperature()) {
            MotorUtil.runMotor(climberMotor, getMotorValue(-CLIMBER_SPEED, CLIMBER_LEFT_FLIPPED));
        //} else {
        //    MotorUtil.runMotor(climberMotor, 0);
        //}
    }

    public void lower() {
       // if (!isDangerousTemperature()) {
            MotorUtil.runMotor(climberMotor, getMotorValue(CLIMBER_SPEED, CLIMBER_LEFT_FLIPPED));
        //} else {
        //    MotorUtil.runMotor(climberMotor, 0);
       // }
    }

    public void zero() {
        this.encoder.reset();
    }

    public double getRotations() {
        return this.encoder.getAbsoluteRotations();
    }

    public boolean isTopLeftSwitchPressed() {
        return tlSwitch.get();
    }

    public boolean isBottomLeftSwitchPressed() {
        return blSwitch.get();
    }
}
