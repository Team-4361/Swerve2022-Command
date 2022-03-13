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

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
    private final CANSparkMax[] sparks;

    private final Motor extender;
    private final Motor intakeMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final DigitalInput flMagnet;
    private final DigitalInput frMagnet;
    private final DigitalInput blMagnet;
    private final DigitalInput brMagnet;

    private final ProportionalController intakeController = new ProportionalController(/*0.09*/ 0.01);

    public IntakeSubsystem() {
        CANSparkMax leftSpark = new CANSparkMax(L_INTAKE_EXTEND_ID, kBrushless);
        CANSparkMax rightSpark = new CANSparkMax(R_INTAKE_EXTEND_ID, kBrushless);
        CANSparkMax intakeSpark = new CANSparkMax(INTAKE_SPIN_MOTOR_ID, kBrushed);
        sparks = new CANSparkMax[]{leftSpark, rightSpark, intakeSpark};

        extender = new MultiMotor(
                new AbstractMotor(
                        leftSpark::set,
                        leftSpark::get,
                        MotorFlip.INTAKE_EXTENDER_LEFT_FLIPPED
                ),
                new AbstractMotor(
                        rightSpark::set,
                        rightSpark::get,
                        MotorFlip.INTAKE_EXTENDER_RIGHT_FLIPPED
                )
        );
        intakeMotor = new AbstractMotor(
                intakeSpark::set,
                intakeSpark::get,
                MotorFlip.INTAKE_FLIPPED
        );

        leftEncoder = sparks[0].getEncoder();
        rightEncoder = sparks[1].getEncoder();

        // TODO: Not sure if these are reversed or not, needs testing, assuming
        // TODO: front is towards the robot's front, extended from chassis, and back
        // TODO: is inside the robot.
        flMagnet = new DigitalInput(FL_MAGNET_ID);
        frMagnet = new DigitalInput(FR_MAGNET_ID);
        blMagnet = new DigitalInput(BL_MAGNET_ID);
        brMagnet = new DigitalInput(BR_MAGNET_ID);
    }

    

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake: Retracted", isRetracted());
        SmartDashboard.putBoolean("Intake: Extended", isExtended());

        SmartDashboard.putNumber("Intake: Left Position", leftEncoder.getPosition());
        SmartDashboard.putNumber("Intake: Right Position", rightEncoder.getPosition());

        SmartDashboard.putNumber("Intake: Average Position", getAveragePosition());

        SmartDashboard.putNumber("Intake: Position Difference", getDifference());
    }

    public void extendIntake() {
        translateExtender(MotorValue.ACCEPT_SPEED);
    }

    public void retractIntake() {
        translateExtender(-MotorValue.ACCEPT_SPEED);
    }

    public void extendIntake(double speed) {
        translateExtender(speed);
    }

    public void retractIntake(double speed) {
        translateExtender(-speed);
    }

    public void retractIntakePID(){
        translateExtender(intakeController.calculate(getAveragePosition(), INTAKE_RETRACT_SETPOINT));
    }

    public void extendIntakePID(){
        translateExtender(intakeController.calculate(getAveragePosition(), INTAKE_EXTEND_SETPOINT));
    }

    /**
     * @return If both front switches are being pressed in
     */
    public boolean isExtended() {
        return flMagnet.get() && frMagnet.get();
    }

    /**
     * @return If both rear switches are being pressed in
     */
    public boolean isRetracted() {
        return blMagnet.get() && brMagnet.get();
    }

    public void spinIntakeAccept() {
        intakeMotor.setPower(MotorValue.SPIN_INTAKE_ACCEPT);
    }

    public void spinIntakeReject() {
        intakeMotor.setPower(-MotorValue.ACCEPT_SPEED);
    }

    public void stopIntakeGroup() {
        extender.setPower(0);
        intakeMotor.setPower(0);
    }

    private void translateExtender(double value) {
        extender.setPower(value);
    }

    public double getLeftPosition() {
        return leftEncoder.getPosition();
    }

    public double getRightPosition() {
        return rightEncoder.getPosition();
    }

    public double getAveragePosition() {
        return Average.of(getLeftPosition(), getRightPosition());
    }

    public double getAverageAbsolutePosition() {
        return Average.of(leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    public double getDifference() {
        return Math.abs(getLeftPosition()-getRightPosition());
    }

    public void resetEncoders() {
        this.leftEncoder.setPosition(0);
        this.rightEncoder.setPosition(0);
    }

    @Override
    public void close() {
        for (CANSparkMax spark : sparks)
            spark.close();
    }
}
