package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.robot_utils.MotorUtil;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Climber.*;
import static frc.robot.robot_utils.MotorUtil.*;
import static frc.robot.Constants.MotorValue.*;
import static frc.robot.Constants.MotorFlip.*;

public class ClimberSubsystem extends SubsystemBase {

    private final CANSparkMax leftClimberMTR = new CANSparkMax(L_CLIMBER_PORT, kBrushless);
    private final CANSparkMax rightClimberMTR = new CANSparkMax(R_CLIMBER_PORT, kBrushless);

    private final DigitalInput bottomProximitySwitch = new DigitalInput(B_CLIMBER_SWITCH);
    private final DigitalInput topProximitySwitch = new DigitalInput(T_CLIMBER_SWITCH);

    private final CANSparkMax[] climberGroup = new CANSparkMax[]{leftClimberMTR, rightClimberMTR};

    public ClimberSubsystem() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void moveClimberUp() {
        if (!topProximitySwitch.get()) {
            runMotors(climberGroup, getMotorValue(CLIMBER_SPEED, CLIMBER_FLIPPED));
        }
    }

    public void moveClimberDown() {
        if (!bottomProximitySwitch.get()) {
            runMotors(climberGroup, getMotorValue(-CLIMBER_SPEED, CLIMBER_FLIPPED));
        }
    }

    public void stopClimber() {
        stopMotors(climberGroup);
    }

    public void lowerClimber() {
        runMotors(climberGroup, getMotorValue(-CLIMBER_SPEED, CLIMBER_FLIPPED));

        while (!bottomProximitySwitch.get() && !isAnyStalled(climberGroup)) {}

        stopMotors(climberGroup);
    }

    public void raiseClimber() {
        runMotors(climberGroup, getMotorValue(CLIMBER_SPEED, CLIMBER_FLIPPED));

        while (!topProximitySwitch.get() && !isAnyStalled(climberGroup)) {}

        stopMotors(climberGroup);
    }
}
