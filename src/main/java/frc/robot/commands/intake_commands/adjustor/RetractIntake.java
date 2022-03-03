package frc.robot.commands.intake_commands.adjustor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.MotorValue.ADJUSTOR_SPEED;

/**
 * This is designed to be used for normal use of retracting the intake. This works by
 * retracting the intake at the specified speed inside {@link frc.robot.Constants},
 * for the specified amount of rotations, until it reaches the limit. After that,
 * it will run the {@link RetractIntake} command at a low speed, to ensure it
 * reaches the magnets, and is fully retracted without breaking anything.
 */
public class RetractIntake extends CommandBase {

    private final double totalRetractRotations;
    private final boolean onlyRotations;

    /**
     * Preferably everything should have already been reset when retracted,
     * if not this is will prevent any issues.
     */
    private void reset() {
        if (Robot.intake.isRetracted()) {
            Robot.intake.resetEncoders();
        }
    }

    public RetractIntake() {
        this.totalRetractRotations = INTAKE_ROTATION_BUFFER;
        this.onlyRotations = false;
    }

    public RetractIntake(boolean rotations) {
        this.totalRetractRotations = INTAKE_ROTATION_BUFFER;
        this.onlyRotations = rotations;
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
        if (avg > this.totalRetractRotations) {
            Robot.intake.retractIntake(ADJUSTOR_SPEED);
        } else if (avg < this.totalRetractRotations) {
            // We have reached the maximum amount of rotations at full speed, run the
            // ExtendIntakeLimit command at a LOW speed, to prevent issues.
            if (TEST_SAFETY_ENABLED) {
                Robot.intake.stopIntakeGroup();
            }

            if (!onlyRotations) {
                new RetractIntakeMagnet(0.15).schedule();
            }

            end(false);
        }
    }

    @Override
    public boolean isFinished() {
        return Robot.intake.getAveragePosition() <= this.totalRetractRotations;
    }
}

