package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorFlip;
import frc.robot.Constants.MotorValue;
import frc.robot.robot_utils.encoder.ConcurrentRotationalEncoder;
import me.wobblyyyy.pathfinder2.math.Average;
import me.wobblyyyy.pathfinder2.robot.components.AbstractMotor;
import me.wobblyyyy.pathfinder2.robot.components.Motor;
import me.wobblyyyy.pathfinder2.robot.components.MultiMotor;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Intake.*;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
    private final CANSparkMax[] sparks;

    private final Motor extender;
    private final Motor intakeMotor;

    private final ConcurrentRotationalEncoder leftEncoder;
    private final ConcurrentRotationalEncoder rightEncoder;

    private final DigitalInput flLimit;
    private final DigitalInput frLimit;
    private final DigitalInput blLimit;
    private final DigitalInput brLimit;

    public IntakeSubsystem() {
        CANSparkMax leftSpark = new CANSparkMax(L_INTAKE_EXTEND_ID, kBrushless);
        CANSparkMax rightSpark = new CANSparkMax(R_INTAKE_EXTEND_ID, kBrushless);
        CANSparkMax intakeSpark = new CANSparkMax(INTAKE_SPIN_MOTOR_ID, kBrushless);
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

        leftEncoder = new ConcurrentRotationalEncoder(leftSpark).setFlipped(MotorFlip.INTAKE_FLIPPED);
        rightEncoder = new ConcurrentRotationalEncoder(rightSpark).setFlipped(MotorFlip.INTAKE_FLIPPED);

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
        SmartDashboard.putBoolean("Back Switch Pressed:", isRetracted());
        SmartDashboard.putBoolean("Front Switch Pressed:", isExtended());

        SmartDashboard.putNumber("Left Intake Position", getLeftPosition());
        SmartDashboard.putNumber("Right Intake Position", getRightPosition());

        SmartDashboard.putNumber("Intake Position Average", getAveragePosition());

        // Update the encoders
        leftEncoder.periodic();
        rightEncoder.periodic();
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

    /**
     * @return If both front switches are being pressed in
     */
    public boolean isExtended() {
        return flLimit.get() && frLimit.get();
    }

    /**
     * @return If both rear switches are being pressed in
     */
    public boolean isRetracted() {
        return blLimit.get() && brLimit.get();
    }

    public void spinIntakeAccept() {
        intakeMotor.setPower(MotorValue.ACCEPT_SPEED);
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
        return leftEncoder.getAbsoluteRotations();
    }

    public double getRightPosition() {
        return rightEncoder.getAbsoluteRotations();
    }

    public double getAveragePosition() {
        return Average.of(getLeftPosition(), getRightPosition());
    }

    public void resetEncoders() {
        this.leftEncoder.reset();
        this.rightEncoder.reset();
    }

    @Override
    public void close() {
        for (CANSparkMax spark : sparks)
            spark.close();
    }
}
