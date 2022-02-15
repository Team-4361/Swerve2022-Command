package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeShooter.L_INTAKE_MOTOR_ID;
import static frc.robot.Constants.IntakeShooter.R_INTAKE_MOTOR_ID;
import static frc.robot.Constants.IntakeShooter.INTAKE_MOTOR_ID;
import static frc.robot.Constants.IntakeShooter.LENGTH_ROD_TO_ANGULAR_POS;

import static frc.robot.Constants.Climber.*;
import static frc.robot.robot_utils.MotorUtil.*;
import static frc.robot.Constants.MotorValue.*;
import static frc.robot.Constants.MotorFlip.*;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax leftIntakeTransMTR = new CANSparkMax(L_INTAKE_MOTOR_ID, kBrushless);
    private final CANSparkMax rightIntakeTransMTR = new CANSparkMax(R_INTAKE_MOTOR_ID, kBrushless);
    private final CANSparkMax intakeMTR = new CANSparkMax(INTAKE_MOTOR_ID, kBrushless);

    private final CANSparkMax[] intakeMotors = new CANSparkMax[]{leftIntakeTransMTR, rightIntakeTransMTR};

    private final RelativeEncoder leftIntakeTransEncoder;

    private final PIDController intakeController = new PIDController(0.1, 0, 0);

    public IntakeSubsystem() {
        leftIntakeTransEncoder = leftIntakeTransMTR.getEncoder();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void extendIntake() {
        translateIntake(intakeController.calculate(getPosition(), LENGTH_ROD_TO_ANGULAR_POS));
    }

    public void retractIntake() {
        translateIntake(intakeController.calculate(getPosition(), 0));
    }

    public void moveIntakeOut() {
        runMotors(intakeMotors, 0.5);
    }

    public void moveIntakeIn() {
        runMotors(intakeMotors, -0.5);
    }

    public void runIntakeIn() {
        runMotor(intakeMTR, 0.5);
    }

    public void runIntakeOut() {
        runMotor(intakeMTR, -0.5);
    }

    public void stopIntake() {
        stopMotors(intakeMotors);
        stopMotor(intakeMTR);
    }

    public void translateIntake(double value) {
        runMotors(intakeMotors, value);
    }

    public double getPosition() {
        return leftIntakeTransEncoder.getPosition();
    }
}
