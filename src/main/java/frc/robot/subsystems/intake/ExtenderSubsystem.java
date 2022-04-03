package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.robot_utils.motor.MotorUtil;

import static frc.robot.Constants.MotorFlip.*;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.Intake.BR_MAGNET_ID;
import static frc.robot.Constants.MotorValue.ACCEPT_SPEED;
import static frc.robot.robot_utils.motor.MotorUtil.getMotorValue;

public class ExtenderSubsystem extends SubsystemBase {
    private final CANSparkMax leftExtender, rightExtender;
    private final DigitalInput blSensor, brSensor, flSensor, frSensor;

    public ExtenderSubsystem() {
        this.leftExtender = new CANSparkMax(L_INTAKE_EXTEND_ID, kBrushless);
        this.rightExtender = new CANSparkMax(R_INTAKE_EXTEND_ID, kBrushless);

        flSensor = new DigitalInput(FL_MAGNET_ID);
        frSensor = new DigitalInput(FR_MAGNET_ID);
        blSensor = new DigitalInput(BL_MAGNET_ID);
        brSensor = new DigitalInput(BR_MAGNET_ID);
    }

    public void extendIntake() {
        leftExtender.set(MotorUtil.getMotorValue(ACCEPT_SPEED, INTAKE_EXTENDER_LEFT_FLIPPED));
        rightExtender.set(MotorUtil.getMotorValue(ACCEPT_SPEED, INTAKE_EXTENDER_RIGHT_FLIPPED));
    }

    public void retractIntake() {
        leftExtender.set(MotorUtil.getMotorValue(-ACCEPT_SPEED, INTAKE_EXTENDER_LEFT_FLIPPED));
        rightExtender.set(MotorUtil.getMotorValue(-ACCEPT_SPEED, INTAKE_EXTENDER_RIGHT_FLIPPED));
    }

    public void retractIntake(double speed) {
        leftExtender.set(MotorUtil.getMotorValue(-speed, INTAKE_EXTENDER_LEFT_FLIPPED));
        rightExtender.set(MotorUtil.getMotorValue(-speed, INTAKE_EXTENDER_RIGHT_FLIPPED));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intake extended", isFullyExtended());
        SmartDashboard.putBoolean("intake retracted", isFullyRetracted());
    }

    public boolean isFullyExtended() {
        return flSensor.get() && frSensor.get();
    }

    public boolean isFullyRetracted() {
        return blSensor.get() && brSensor.get();
    }

    public void stop() {
        leftExtender.set(0);
        rightExtender.set(0);
    }
}
