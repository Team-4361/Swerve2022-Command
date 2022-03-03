package frc.robot.commands.intake_commands.adjustor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MotorValue;
import frc.robot.Robot;

/**
 * This is only designed to be used at slow speeds, as it completely ignores
 * all encoder values, such as for calibration purposes. Otherwise, it will
 * very quickly destroy any magnets you throw at it...
 *
 * WARNING: Use at slow speeds only! (under 0.3)
 */
public class ExtendIntakeMagnet extends CommandBase {
    private final double extendSpeed;

    @Override
    public void initialize() {
        addRequirements(Robot.intake);
    }

    /** Creates a new {@link ExtendIntakeMagnet} with the default speed. */
    public ExtendIntakeMagnet() {
        this.extendSpeed = MotorValue.ADJUSTOR_SPEED;
    }

    /** Creates a new {@link ExtendIntakeMagnet} with a specified speed, from -1.0 to +1.0 */
    public ExtendIntakeMagnet(double speed) {
        this.extendSpeed = speed;
    }

    @Override
    public void execute() {
        // This runs repeatedly until the command is ended.
        if (!Robot.intake.isExtended()) {
            // While the front switch is not pressed, keep running the Intake Extender Motor out.
            Robot.intake.extendIntake(extendSpeed);
        } else {
            // The magnet is pressed, stop the intake and end the command.
            Robot.intake.stopIntakeGroup();
            end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.stopIntakeGroup();
    }

    @Override
    public boolean isFinished() {
        // Will be finished when the front switch is pressed, meaning all the way extended.
        return Robot.intake.isExtended();
    }
}