package frc.robot.commands.test_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ChassisForwardOffsetTest extends CommandBase {

    private final PIDController chassisController;

    public ChassisForwardOffsetTest() {
        chassisController = new PIDController(0.1, 0, 0);
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Drive forward while the command is being executed.
        Robot.swerveDrive.driveForward();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}