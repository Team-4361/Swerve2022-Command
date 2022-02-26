package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorFlip;
import me.wobblyyyy.pathfinder2.revrobotics.SparkMaxMotor;
import me.wobblyyyy.pathfinder2.robot.components.Motor;
import me.wobblyyyy.pathfinder2.robot.components.MultiMotor;
import frc.robot.robot_utils.encoder.RotationalAbsoluteEncoder;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Climber.*;
import static frc.robot.Constants.MotorValue.CLIMBER_SPEED;
import static frc.robot.robot_utils.MotorUtil.*;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax leftClimberMTR = new CANSparkMax(L_CLIMBER_ID, kBrushless);
    private final CANSparkMax rightClimberMTR = new CANSparkMax(R_CLIMBER_ID, kBrushless);

    private final Motor climber;

    private final DigitalInput blSwitch;
    private final DigitalInput brSwitch;
    private final DigitalInput tlSwitch;
    private final DigitalInput trSwitch;

    private final CANSparkMax[] climberGroup = new CANSparkMax[]{leftClimberMTR, rightClimberMTR};

    private final RotationalAbsoluteEncoder leftEncoder = new RotationalAbsoluteEncoder(leftClimberMTR)
        .setFlipped(MotorFlip.CLIMBER_LEFT_FLIPPED)
        .start();

    public final RotationalAbsoluteEncoder rightEncoder = new RotationalAbsoluteEncoder(rightClimberMTR)
        .setFlipped(MotorFlip.CLIMBER_RIGHT_FLIPPED)
        .start();

    public ClimberSubsystem() {
        Motor leftMotor = new SparkMaxMotor(L_CLIMBER_ID, kBrushless);
        Motor rightMotor = new SparkMaxMotor(R_CLIMBER_ID, kBrushless);

        if (MotorFlip.CLIMBER_LEFT_FLIPPED)
            leftMotor = leftMotor.invert();
        if (MotorFlip.CLIMBER_RIGHT_FLIPPED)
            rightMotor = rightMotor.invert();

        climber = new MultiMotor(leftMotor, rightMotor);

        blSwitch = new DigitalInput(BL_LIMIT_ID);
        brSwitch = new DigitalInput(BR_LIMIT_ID);
        tlSwitch = new DigitalInput(TL_LIMIT_ID);
        trSwitch = new DigitalInput(TR_LIMIT_ID);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("left climber encoder:", leftEncoder.getAbsoluteRotations());
        SmartDashboard.putNumber("right climber encoder:", rightEncoder.getAbsoluteRotations());

        SmartDashboard.putBoolean("bl switch:", blSwitch.get());
        SmartDashboard.putBoolean("br switch:", brSwitch.get());
        SmartDashboard.putBoolean("tl switch:", tlSwitch.get());
        SmartDashboard.putBoolean("tr switch:", trSwitch.get());
    }

    public void zero() {
        leftEncoder.resetZero();
        rightEncoder.resetZero();
    }

    public void stopClimber() {
        stopMotors(climberGroup);
    }


    public void raiseLeftClimber() {
        translateLeftClimber(-CLIMBER_SPEED);
    }

    public void raiseRightClimber() {
        translateRightClimber(-CLIMBER_SPEED);
    }

    public void lowerLeftClimber() {
        translateLeftClimber(CLIMBER_SPEED);
    }

    public void lowerRightClimber() {
        translateRightClimber(CLIMBER_SPEED);
    }

    public void translateClimber(double value) {
        // make sure the climber doesn't move up if it can't move up
        // or move down if it can't move down
        if (value > 0) {
            if (isTopSwitchPressed()) {
                climber.setPower(0);
            } else {
                climber.setPower(value);
            }
        } else if (value < 0) {
            if (isBottomSwitchPressed()) {
                climber.setPower(0);
            } else {
                climber.setPower(value);
            }
        } else {
            climber.setPower(value);
        }
    }

    public void translateLeftClimber(double value) {
        runMotor(leftClimberMTR,
                getMotorValue(value, MotorFlip.CLIMBER_LEFT_FLIPPED));
    }

    public void translateRightClimber(double value) {
        runMotor(rightClimberMTR,
                getMotorValue(value, MotorFlip.CLIMBER_RIGHT_FLIPPED));
    }

    /**
     * @return If the top switch is pressed
     */
    public boolean isTopSwitchPressed() {
        return tlSwitch.get() || trSwitch.get();
    }

    public boolean isTopLeftSwitchPressed() {
        return tlSwitch.get();
    }

    public void stopLeftClimber(){
        translateLeftClimber(0);
    }

    public void stopRightClimber(){
        translateRightClimber(0);
    }

    /**
     * @return If the top switch is pressed
     */
    public boolean isTopRightSwitchPressed() {
        return trSwitch.get();
    }

    /**
     * @return If the top switch is pressed
     */
    public boolean isBottomLeftSwitchPressed() {
        return blSwitch.get();
    }

    /**
     * @return If the top switch is pressed
     */
    public boolean isBottomSwitchPressed() {
        return blSwitch.get() || brSwitch.get();
    }

    public boolean isBottomRightSwitchPressed() {
        return brSwitch.get();
    }
}
