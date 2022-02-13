package frc.robot.commands.chassis_commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;

public class ToggleLeftHandMode extends CommandBase {

    private boolean finished;

    @Override
    public void initialize() {
        finished = false;
        SmartDashboard.putBoolean("Robot: Left Handed", Robot.leftHandedMode);
    }


    @Override
    public void execute() {
        Robot.leftHandedMode = !Robot.leftHandedMode;
        SmartDashboard.putBoolean("Robot: Left Handed", Robot.leftHandedMode);
        finished = true;
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        // Returns true when the Left Handed Mode has been changed.
        return !finished;
    }
}