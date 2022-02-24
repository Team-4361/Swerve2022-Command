package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorFlip;
import frc.robot.Constants.MotorValue;
import frc.robot.robot_utils.encoder.RotationalAbsoluteEncoder;

import static frc.robot.Constants.Intake.*;
import static frc.robot.robot_utils.MotorUtil.*;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax leftIntakeTransMTR;
    private final CANSparkMax rightIntakeTransMTR;
    private final CANSparkMax intakeMTR;

    private final CANSparkMax[] intakeMotors;

    private final RotationalAbsoluteEncoder leftEncoder, rightEncoder;
    private final DigitalInput flLimit, frLimit, blLimit, brLimit;

    private final PIDController intakeController = new PIDController(0.1, 0, 0);

    public IntakeSubsystem() {
        leftIntakeTransMTR = new CANSparkMax(L_INTAKE_EXTEND_ID, kBrushless);
        rightIntakeTransMTR = new CANSparkMax(R_INTAKE_EXTEND_ID, kBrushless);
        intakeMTR = new CANSparkMax(INTAKE_SPIN_MOTOR_ID, kBrushless);

        intakeMotors = new CANSparkMax[]{leftIntakeTransMTR, rightIntakeTransMTR};

        leftEncoder = new RotationalAbsoluteEncoder(leftIntakeTransMTR)
            .setAccuracyFactor(5)
            .setFlipped(MotorFlip.INTAKE_FLIPPED)
            .start();

        rightEncoder = new RotationalAbsoluteEncoder(rightIntakeTransMTR)
            .setAccuracyFactor(5)
            .setFlipped(MotorFlip.INTAKE_FLIPPED)
            .start();

        // TODO: Not sure if these are reversed or not, needs testing, assuming
        // TODO: front is towards the robot's front, extended from chassis, and back
        // TODO: is inside the robot.
        flLimit = new DigitalInput(FL_LIMIT_ID);
        frLimit = new DigitalInput(FR_LIMIT_ID);

        blLimit = new DigitalInput(BL_LIMIT_ID);
        brLimit = new DigitalInput(BR_LIMIT_ID);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("BR_Limit", brLimit.get());
        SmartDashboard.putBoolean("FR_Limit", frLimit.get());
    }

    /** extends the intake automatically */
    public void extendIntake() {
        if (LIMIT_SWITCH_ENABLED) {
            translateIntake(MotorValue.ACCEPT_SPEED);

            while (!flLimit.get() && !frLimit.get()) {
                // Wait for either one of the robot's intake switches to be detected.
                Thread.onSpinWait();
            }
        } else {
            // Good backup solution
            leftEncoder.setMotorRotations(INTAKE_EXTEND_ROTATIONS, MotorValue.ACCEPT_SPEED);
            rightEncoder.setMotorRotations(INTAKE_EXTEND_ROTATIONS, MotorValue.ACCEPT_SPEED);
        }

        stopIntake();
    }

    public void retractIntake() {
        if (LIMIT_SWITCH_ENABLED) {
            translateIntake(-MotorValue.ACCEPT_SPEED);

            while (!blLimit.get() && !brLimit.get()) {
                // Wait for either one of the robot's intake switches to be detected.
                Thread.onSpinWait();
            }
        } else {
            // Good backup solution
            leftEncoder.setMotorRotations(INTAKE_EXTEND_ROTATIONS, -MotorValue.ACCEPT_SPEED);
            rightEncoder.setMotorRotations(INTAKE_EXTEND_ROTATIONS, -MotorValue.ACCEPT_SPEED);
        }

        stopIntake();
    }

    public void moveIntakeOut() {
        runMotors(intakeMotors, getMotorValue(MotorValue.ACCEPT_SPEED, MotorFlip.INTAKE_EXTENDER_FLIPPED));
    }

    public void moveIntakeIn() {
        runMotors(intakeMotors, getMotorValue(-MotorValue.ACCEPT_SPEED, MotorFlip.INTAKE_EXTENDER_FLIPPED));
    }

    public void runIntakeIn() {
        runMotor(intakeMTR, getMotorValue(MotorValue.ACCEPT_SPEED, MotorFlip.INTAKE_FLIPPED));
    }

    public void runIntakeOut() {
        runMotor(intakeMTR, getMotorValue(-MotorValue.ACCEPT_SPEED, MotorFlip.INTAKE_FLIPPED));
    }

    public void stopIntake() {
        stopMotors(intakeMotors);
        stopMotor(intakeMTR);
    }

    public void translateIntake(double value) {
        runMotors(intakeMotors, getMotorValue(value, MotorFlip.INTAKE_FLIPPED));
    }

    public double getPosition() {
        return leftEncoder.getAbsoluteRotations();
    }
}
