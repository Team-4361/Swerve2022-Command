package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorFlip;
import me.wobblyyyy.pathfinder2.revrobotics.SparkMaxMotor;
import me.wobblyyyy.pathfinder2.robot.components.Motor;
import me.wobblyyyy.pathfinder2.robot.components.MultiMotor;

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
    }

    public void stopClimber() {
        stopMotors(climberGroup);
    }

    public void lowerClimber() {
        translateClimber(-CLIMBER_SPEED);
    }

    public void raiseClimber() {
        translateClimber(CLIMBER_SPEED);
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

    /**
     * @return If the top switch is pressed
     */
    public boolean isTopSwitchPressed() {
        return tlSwitch.get() || trSwitch.get();
    }

    /**
     * @return If the bottom switch is pressed
     */
    public boolean isBottomSwitchPressed() {
        return blSwitch.get() || brSwitch.get();
    }
}
