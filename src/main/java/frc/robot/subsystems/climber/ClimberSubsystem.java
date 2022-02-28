package frc.robot.subsystems.climber;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Climber.BL_LIMIT_ID;
import static frc.robot.Constants.Climber.BR_LIMIT_ID;
import static frc.robot.Constants.Climber.L_CLIMBER_ID;
import static frc.robot.Constants.Climber.R_CLIMBER_ID;
import static frc.robot.Constants.Climber.TL_LIMIT_ID;
import static frc.robot.Constants.Climber.TR_LIMIT_ID;
import static frc.robot.Constants.MotorFlip.CLIMBER_LEFT_FLIPPED;
import static frc.robot.Constants.MotorFlip.CLIMBER_RIGHT_FLIPPED;
import static frc.robot.Constants.MotorValue.CLIMBER_SPEED;
import static frc.robot.robot_utils.MotorUtil.getMotorValue;
import static frc.robot.robot_utils.MotorUtil.runMotor;
import static frc.robot.robot_utils.MotorUtil.stopMotors;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorFlip;

import frc.robot.robot_utils.encoder.*;

public class ClimberSubsystem extends SubsystemBase {

    private final CANSparkMax leftClimberMTR = new CANSparkMax(L_CLIMBER_ID, kBrushless);
    private final CANSparkMax rightClimberMTR = new CANSparkMax(R_CLIMBER_ID, kBrushless);

    private final DigitalInput blSwitch, brSwitch, tlSwitch, trSwitch;

    private final CANSparkMax[] climberGroup = new CANSparkMax[]{leftClimberMTR, rightClimberMTR};

    private final RotationalAbsoluteEncoder leftEncoder = new RotationalAbsoluteEncoder(leftClimberMTR)
            .setFlipped(MotorFlip.CLIMBER_LEFT_FLIPPED);

    public final RotationalAbsoluteEncoder rightEncoder = new RotationalAbsoluteEncoder(rightClimberMTR)
            .setFlipped(MotorFlip.CLIMBER_RIGHT_FLIPPED);

    public ClimberSubsystem() {
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

        leftEncoder.update();
        rightEncoder.update();
    }

    public void zero() {
        leftEncoder.resetZero();
        rightEncoder.resetZero();
    }

    public void stopClimber() {stopMotors(climberGroup);}


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

    public void translateLeftClimber(double value) {
        runMotor(leftClimberMTR, getMotorValue(value, CLIMBER_LEFT_FLIPPED));
    }

    public void translateRightClimber(double value) {
        runMotor(rightClimberMTR, getMotorValue(value, CLIMBER_RIGHT_FLIPPED));
    }

    /**
     * @return If the top switch is pressed
     */
    public boolean isTopLeftSwitchPressed() {
        return tlSwitch.get();
    }

    public void stopLeftClimber() {
        translateLeftClimber(0);
    }

    public void stopRightClimber() {
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
    public boolean isBottomRightSwitchPressed() {
        return brSwitch.get();
    }
}
