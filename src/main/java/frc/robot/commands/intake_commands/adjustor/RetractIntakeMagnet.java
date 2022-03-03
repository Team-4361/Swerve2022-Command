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
public class RetractIntakeMagnet extends CommandBase {
    private final double retractSpeed;

    @Override
    public void initialize() {
        addRequirements(Robot.intake);
    }

    /** Creates a new {@link RetractIntakeMagnet} with the default speed. */
    public RetractIntakeMagnet() {
        this.retractSpeed = MotorValue.ADJUSTOR_SPEED;
    }

    /** Creates a new {@link RetractIntakeMagnet} with a specified speed, from +0 to +1.0 */
    public RetractIntakeMagnet(double speed) {
        this.retractSpeed = speed;
    }

    @Override
    public void execute() {
        // This runs repeatedly until the command is ended.
        if (!Robot.intake.isRetracted()) {
            // While the rear switch is not pressed, keep running the Intake Retract Motor out.
            //Robot.intake.retractIntake(retractSpeed);
            Robot.intake.retractIntakePID();
        } else {
            // The magnet is pressed, stop the intake and end the command.
            Robot.intake.stopIntakeGroup();

            // Reset the encoders since we know we're at the end.
            Robot.intake.resetEncoders();

            end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // The stop intake method disables retract motors, as well as the accept/reject
        // motors. This is okay in this scenario because we don't want to be taking in balls when
        // the intake is retracted.
        Robot.intake.stopIntakeGroup();
    }

    @Override
    public boolean isFinished() {
        // Will be finished when the front switch is pressed, meaning all the way extended.
        return Robot.intake.isRetracted() ||  (Robot.intake.getLeftPosition() >= -0.5 && Robot.intake.getRightPosition() >= -0.5);
    }
}