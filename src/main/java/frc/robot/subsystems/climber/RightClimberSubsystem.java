package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot_utils.motor.MotorUtil;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Climber.*;
import static frc.robot.Constants.MotorFlip.CLIMBER_LEFT_FLIPPED;
import static frc.robot.Constants.MotorValue.CLIMBER_SPEED;
import static frc.robot.robot_utils.motor.MotorUtil.getMotorValue;

public class RightClimberSubsystem extends SubsystemBase {
    private final CANSparkMax climberMotor = new CANSparkMax(R_CLIMBER_ID, kBrushless);
    private final Debouncer debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    private final DigitalInput brSwitch, trSwitch;

    public RightClimberSubsystem() {
        this.brSwitch = new DigitalInput(BR_LIMIT_ID);
        this.trSwitch = new DigitalInput(TR_LIMIT_ID);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("br switch", brSwitch.get());
        SmartDashboard.putBoolean("tr switch", trSwitch.get());
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

    public boolean isTopRightSwitchPressed() {
        return debouncer.calculate(trSwitch.get());
    }

    public boolean isBottomRightSwitchPressed() {
        return debouncer.calculate(brSwitch.get());
    }
}
