package frc.robot.commands.intake_commands.adjustor;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.Intake.INTAKE_EXTEND_SETPOINT;

public class ExtendIntakeMagnet extends CommandBase {
    @Override
    public void initialize() {
        addRequirements(Robot.intake);
        System.out.println("Extending Intake");
    }

    @Override
    public void execute() {
        // This runs repeatedly until the command is ended.
        if (!Robot.intake.isExtended()) {
            // While the front switch is not pressed, keep running the Intake Extender Motor out.
            Robot.intake.extendIntake();
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
        return Robot.intake.isExtended() || (Robot.intake.getLeftPosition() <= INTAKE_EXTEND_SETPOINT && Robot.intake.getRightPosition() <= INTAKE_EXTEND_SETPOINT);
    }
}