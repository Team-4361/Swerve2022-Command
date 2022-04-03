package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorFlip;
import frc.robot.Constants.MotorValue;
import me.wobblyyyy.pathfinder2.control.ProportionalController;
import me.wobblyyyy.pathfinder2.math.Average;
import me.wobblyyyy.pathfinder2.robot.components.AbstractMotor;
import me.wobblyyyy.pathfinder2.robot.components.Motor;
import me.wobblyyyy.pathfinder2.robot.components.MultiMotor;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

import static frc.robot.Constants.Intake.*;

public class IntakeSubsystem extends SubsystemBase {
    private final Motor intakeMotor;

    private final ProportionalController intakeController = new ProportionalController(/*0.09*/ 0.01);

    public IntakeSubsystem() {
        //CANSparkMax leftSpark = new CANSparkMax(L_INTAKE_EXTEND_ID, kBrushless);
        //CANSparkMax rightSpark = new CANSparkMax(R_INTAKE_EXTEND_ID, kBrushless);
        CANSparkMax intakeSpark = new CANSparkMax(INTAKE_SPIN_MOTOR_ID, kBrushed);

        //sparks = new CANSparkMax[]{leftSpark, rightSpark, intakeSpark};

        intakeMotor = new AbstractMotor(
                intakeSpark::set,
                intakeSpark::get,
                MotorFlip.INTAKE_FLIPPED
        );
    }

    @Override
    public void periodic() {

        //SmartDashboard.putNumber("Intake: Left Position", leftEncoder.getPosition());
        //SmartDashboard.putNumber("Intake: Right Position", rightEncoder.getPosition());

        //SmartDashboard.putNumber("Intake: Average Position", getAveragePosition());

        //SmartDashboard.putNumber("Intake: Position Difference", getDifference());
    }



    public void spinIntakeAccept() {
        intakeMotor.setPower(MotorValue.SPIN_INTAKE_ACCEPT);
    }

    public void spinIntakeReject() {
        intakeMotor.setPower(-MotorValue.ACCEPT_SPEED);
    }

    public void stopIntakeGroup() {
        intakeMotor.setPower(0);
    }

    /**
     * translate one of the extender's motors, so long as the translation
     * is valid. if the translation would cause the extender to extend past
     * either of its limits (either extend too far or retract too far), the
     * motor will be set a power value of 0 so that the motors don't get
     * burned out.
     *
     * @param motor       the motor to set power to.
     * @param power       the power value to set to the motor.
     * @param isExtended  is the FRONT limit switch activated?
     * @param isRetracted is the BACK limit switch activated?
     */
    private static void translateExtender(Motor motor,
                                          double power,
                                          boolean isExtended,
                                          boolean isRetracted) {
        //if (power > 0 && isExtended) power = 0;
        //else if (power < 0 && isRetracted) power = 0;

        motor.setPower(power);
    }

    public void stopIntakeMotor(){
        intakeMotor.setPower(0);
    }
}
