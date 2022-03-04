package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorFlip;
import frc.robot.Constants.MotorValue;
import frc.robot.commands.intake_commands.adjustor.CalibrateRetractIntake;
import frc.robot.commands.intake_commands.adjustor.RetractIntakeMagnet;
import frc.robot.robot_utils.encoder.ConcurrentRotationalEncoder;
import me.wobblyyyy.pathfinder2.control.ProportionalController;
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

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    // For testing the values that it would have been under absolute
    private final ConcurrentRotationalEncoder leftRotationalEncoder;
    private final ConcurrentRotationalEncoder rightRotationalEncoder;

    private final Debouncer debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

    private final DigitalInput flMagnet;
    private final DigitalInput frMagnet;
    private final DigitalInput blMagnet;
    private final DigitalInput brMagnet;

    private final ProportionalController intakeController = new ProportionalController(0.09);

    public IntakeSubsystem() {
        CANSparkMax leftSpark = new CANSparkMax(L_INTAKE_EXTEND_ID, kBrushless);
        CANSparkMax rightSpark = new CANSparkMax(R_INTAKE_EXTEND_ID, kBrushless);
        CANSparkMax intakeSpark = new CANSparkMax(INTAKE_SPIN_MOTOR_ID, kBrushless);
        sparks = new CANSparkMax[]{leftSpark, rightSpark, intakeSpark};

        leftRotationalEncoder = new ConcurrentRotationalEncoder(leftSpark)
                .setFlipped(MotorFlip.INTAKE_EXTENDER_LEFT_FLIPPED);

        rightRotationalEncoder = new ConcurrentRotationalEncoder(rightSpark)
                .setFlipped(MotorFlip.INTAKE_EXTENDER_RIGHT_FLIPPED);

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

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);


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
        SmartDashboard.putBoolean("Back Switch Pressed:", isRetracted());
        SmartDashboard.putBoolean("Front Switch Pressed:", isExtended());

        SmartDashboard.putNumber("Left Intake Position", leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Intake Position", rightEncoder.getPosition());

        SmartDashboard.putNumber("Intake Position Average", getAveragePosition());

        SmartDashboard.putNumber("Left Absolute Intake", leftRotationalEncoder.getAbsoluteRotations());
        SmartDashboard.putNumber("Right Absolute Intake", rightRotationalEncoder.getAbsoluteRotations());

        SmartDashboard.putNumber("Intake Absolute Position Avg", getAverageAbsolutePosition());

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
        return debouncer.calculate(flMagnet.get()) && debouncer.calculate(frMagnet.get());
    }

    /**
     * @return If both rear switches are being pressed in
     */
    public boolean isRetracted() {
        return debouncer.calculate(blMagnet.get()) && debouncer.calculate(brMagnet.get());
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
        return leftEncoder.getPosition();
    }

    public double getRightPosition() {
        return rightEncoder.getPosition();
    }

    public double getAveragePosition() {
        return Average.of(getLeftPosition(), getRightPosition());
    }

    public double getAverageAbsolutePosition() {
        return Average.of(leftRotationalEncoder.getAbsoluteRotations(), rightRotationalEncoder.getAbsoluteRotations());
    }

    public void resetEncoders() {
        this.leftEncoder.setPosition(0);
        this.rightEncoder.setPosition(0);
    }

    public void calibrate(){
        new CalibrateRetractIntake().schedule();
    }

    @Override
    public void close() {
        for (CANSparkMax spark : sparks)
            spark.close();
    }
}
