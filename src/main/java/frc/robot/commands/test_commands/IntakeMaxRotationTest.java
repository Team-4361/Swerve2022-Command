package frc.robot.commands.test_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.intake_commands.adjustor.ExtendIntakeLimit;
import frc.robot.commands.intake_commands.adjustor.RetractIntakeLimit;

import static frc.robot.Constants.Intake.INTAKE_ROTATION_BUFFER;

/**
 * This is used for testing purposes on the robot, and it fully extends the intake, and records
 * the amount of rotations it takes on the {@link SmartDashboard}. After that, it fully retracts the
 * intake and zeros it out.
 */
public class IntakeMaxRotationTest extends SequentialCommandGroup {

    protected static class RecordInformation extends CommandBase {
        private boolean finished = false;

        @Override
        public void initialize() {
            addRequirements(Robot.intake);

            double position = Robot.intake.getAveragePosition();

            SmartDashboard.putNumber("Test: Total Intake Extension", position);
            SmartDashboard.putNumber("Test: Fast Intake Extension", position-INTAKE_ROTATION_BUFFER);
            SmartDashboard.putNumber("Test: Total Intake Retraction", 0);
            SmartDashboard.putNumber("Test: Fast Intake Retraction", INTAKE_ROTATION_BUFFER);

            this.finished = true;
            end(false);
        }

        @Override
        public void execute() {
            if (this.finished)
                end(false);
        }

        @Override
        public boolean isFinished() {
            return this.finished;
        }
    }

    public IntakeMaxRotationTest() {
        super(
                new ExtendIntakeLimit(0.15),
                new RecordInformation(),
                new RetractIntakeLimit(0.15)
        );
    }
}
