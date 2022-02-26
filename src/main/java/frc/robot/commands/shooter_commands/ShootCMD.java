package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;


public class ShootCMD extends CommandBase {
    public ShootCMD() {
        addRequirements(Robot.shooter);
    }

    @Override
    public void initialize() {

    }


    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        Robot.shooter.setShooterMotor(0);
    }

    @Override
    public boolean isFinished() {
        return Robot.storage.getBallsLoaded() == 0;
    }
}
