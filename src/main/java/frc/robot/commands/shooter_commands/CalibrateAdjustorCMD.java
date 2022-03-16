package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class CalibrateAdjustorCMD extends CommandBase {
    private boolean angleCalibrated = false;

    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        addRequirements(Robot.adjustor);
        this.angleCalibrated = false;
    }

    /** The main body of a command. Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        // Assume the motor is initially started in the downwards position, and slowly raise the motor until
        // the limit switch is pressed, when it can then be calibrated.
        if (!Robot.adjustor.atLimitAngle()) {
            // Move the motor up, so it hits the limit switch.
            Robot.adjustor.translateUp();
        } else {
            // If we did not already calibrate the angle, stop the adjustor and set the max angle (since it will
            // be at the very top from the limit switch).
            if (!angleCalibrated) {
                Robot.adjustor.stop();
                Robot.adjustor.setMaximumAngle(Robot.adjustor.getAngle());
                this.angleCalibrated = true;
            }
        }
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally, or when it
     * interrupted/canceled.
     *
     * <p>Do not schedule commands here that share requirements with this command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        // When the command is ended, set the target angle back to zero degrees, so it moves all the way down.
        Robot.adjustor.setTargetAngle(0);
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end() method and
     * un-schedule it.
     *
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        return this.angleCalibrated;
    }
}
