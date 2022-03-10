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
import static frc.robot.Constants.MotorFlip.CLIMBER_RIGHT_FLIPPED;
import static frc.robot.Constants.MotorValue.CLIMBER_SPEED;
import static frc.robot.robot_utils.motor.MotorUtil.getMotorValue;

public class RightClimberSubsystem extends SubsystemBase {
    private final CANSparkMax climberMotor = new CANSparkMax(R_CLIMBER_ID, kBrushless);
    private final Debouncer debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

    /**
     * This encoder is used for being able to run the climber at a high speed until nearly the max rotations,
     * then start ramping down the speed until the limit switch is hit.
     */
    private final ConcurrentRotationalEncoder encoder;

    private final DigitalInput brSwitch, trSwitch;

    private boolean isDone = false;

    public void setDone(boolean done) {
        this.isDone = done;
    }

    public boolean getDone() {
        return this.isDone;
    }

    public RightClimberSubsystem() {
        this.brSwitch = new DigitalInput(BR_LIMIT_ID);
        this.trSwitch = new DigitalInput(TR_LIMIT_ID);

        this.encoder = new ConcurrentRotationalEncoder(climberMotor)
                .setFlipped(CLIMBER_RIGHT_FLIPPED);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climber: BR Switch", brSwitch.get());
        SmartDashboard.putBoolean("Climber: TR Switch", trSwitch.get());

        SmartDashboard.putNumber("Climber: Right Rotations", getRotations());
        SmartDashboard.putNumber("Climber: Right Motor Temp", climberMotor.getMotorTemperature());
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

    public boolean isTopRightSwitchPressed() {
        return debouncer.calculate(trSwitch.get());
    }

    public boolean isBottomRightSwitchPressed() {
        return debouncer.calculate(brSwitch.get());
    }
}
