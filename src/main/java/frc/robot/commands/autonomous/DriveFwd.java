package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveFwd extends CommandBase {
    @Override
    public void initialize() {
       addRequirements(Robot.swerveDrive);
       Robot.swerveDrive.autoDrive(0,-0.5, 0);
    }

    @Override
    public boolean isFinished() {
        return  true;
    }
}
