package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;

import static frc.robot.Constants.IntakeShooter.MAX_INTAKE_MOTOR_POSITION;

public class UserTransIntakeOut extends CommandBase {
    
    @Override
    public void initialize() {
        addRequirements(Robot.intake);
    }

    
    @Override
    public void execute() {
        if(Robot.intake.getPosition() < MAX_INTAKE_MOTOR_POSITION) {
            Robot.intake.runIntakeOut();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}