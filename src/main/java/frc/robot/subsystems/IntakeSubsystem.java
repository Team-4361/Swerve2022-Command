package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeShooter;
import frc.robot.Constants.MotorFlip;
import frc.robot.Constants.MotorValue;
import frc.robot.robot_utils.encoder.RotationalAbsoluteEncoder;

import static frc.robot.Constants.IntakeShooter.L_INTAKE_MOTOR_ID;
import static frc.robot.Constants.IntakeShooter.R_INTAKE_MOTOR_ID;
import static frc.robot.Constants.IntakeShooter.INTAKE_MOTOR_ID;
import static frc.robot.Constants.IntakeShooter.LENGTH_ROD_TO_ANGULAR_POS;

import static frc.robot.robot_utils.MotorUtil.*;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax leftIntakeTransMTR;
    private final CANSparkMax rightIntakeTransMTR;
    private final CANSparkMax intakeMTR;

    private final CANSparkMax[] intakeMotors;

    private final RotationalAbsoluteEncoder leftEncoder, rightEncoder;

    private final PIDController intakeController = new PIDController(0.1, 0, 0);

    public IntakeSubsystem() {
        leftIntakeTransMTR = new CANSparkMax(L_INTAKE_MOTOR_ID, kBrushless);
        rightIntakeTransMTR = new CANSparkMax(R_INTAKE_MOTOR_ID, kBrushless);
        intakeMTR = new CANSparkMax(INTAKE_MOTOR_ID, kBrushless);

        intakeMotors = new CANSparkMax[]{leftIntakeTransMTR, rightIntakeTransMTR};

        leftEncoder = new RotationalAbsoluteEncoder(leftIntakeTransMTR)
            .setAccuracyFactor(5)
            .setFlipped(MotorFlip.INTAKE_FLIPPED)
            .start();

        rightEncoder = new RotationalAbsoluteEncoder(rightIntakeTransMTR)
            .setAccuracyFactor(5)
            .setFlipped(MotorFlip.INTAKE_FLIPPED)
            .start();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void extendIntake() {
        //translateIntake(intakeController.calculate(getPosition(), LENGTH_ROD_TO_ANGULAR_POS));
        leftEncoder.setMotorRotations(IntakeShooter.INTAKE_EXTEND_ROTATIONS, MotorValue.ACCEPT_SPEED);
    }

    public void retractIntake() {
        //translateIntake(intakeController.calculate(getPosition(), LENGTH_ROD_TO_ANGULAR_POS));
        leftEncoder.setMotorRotations(0, MotorValue.ACCEPT_SPEED);
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
