package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.robot_utils.encoder.ConcurrentRotationalEncoder;

import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.MotorValue.ADJUSTOR_SPEED;

/**
 * This is designed to be used for normal use of extending the intake. This works by
 * extending the intake at the specified speed inside {@link frc.robot.Constants},
 * for the specified amount of rotations, until it reaches the limit. After that,
 * it will run the {@link ExtendIntake} command at a low speed, to ensure it
 * reaches the limit switches, and is fully extended without breaking anything.
 */
public class ExtendIntake extends CommandBase {

    private final double totalExtendRotations;

    /**
     * Preferably everything should have already been reset when retracted,
     * if not this is will prevent any issues.
     */
    private void reset() {
        if (Robot.intake.isRetracted()) {
            Robot.intake.resetEncoders();
        }
    }

    public ExtendIntake() {
        this.totalExtendRotations = INTAKE_TOTAL_EXTEND_ROTATIONS-INTAKE_ROTATION_BUFFER;
    }

    @Override
    public void initialize() {
        addRequirements(Robot.intake);
        reset();
    }

    @Override
    public void execute() {
        double avg = Robot.intake.getAveragePosition();

        // If we haven't reached the rotation limit yet, keep extending at full speed.
        if (avg < this.totalExtendRotations) {
            Robot.intake.extendIntake(ADJUSTOR_SPEED);
        } else if (avg > this.totalExtendRotations) {
            // We have reached the maximum amount of rotations at full speed, run the
            // ExtendIntakeLimit command at a LOW speed, to prevent issues.
            if (TEST_SAFETY_ENABLED) {
                Robot.intake.stopIntakeGroup();
            }
            new ExtendIntakeLimit(0.15).schedule();

            end(false);
        }
    }

    @Override
    public boolean isFinished() {
        return Robot.intake.getAveragePosition() >= this.totalExtendRotations;
    }
}
