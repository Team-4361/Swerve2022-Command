package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.MotorValue.*;

public class ShootCMD extends CommandBase{

    private boolean shouldAddRequirements = true;

    private double shootSpeed;

    /**
     * 
     * @param cmd command using the storage subsystem
     * @param shootSpeed shoot speed in RPM
     */
    public ShootCMD(Command cmd, double shootSpeed){
        if(!cmd.isScheduled()){
            this.cancel();

            shouldAddRequirements = false;

            this.shootSpeed = shootSpeed;
        }
    }

    public ShootCMD(double shootSpeed){
        this.shootSpeed = shootSpeed;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        if(shouldAddRequirements){
            addRequirements(Robot.storage, Robot.shooter); 
        }
    }

    @Override
    public void execute() {
        // Only run the rear storage motor when the shooter has reached its target RPM.
        if (Robot.shooter.isDesiredSpeed(this.shootSpeed)) {
            // Run the storage motor, due to the if statement above it will shut off when the ball leaves.
            Robot.storage.setStorageMotor(SLOW_ACCEPT_SPEED);
            Robot.shooter.setShooterVelocity(this.shootSpeed);
        } else{
            Robot.storage.setStorageMotor(0);
            Robot.shooter.setShooterVelocity(this.shootSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        Robot.storage.setAcceptorMotor(0);
        Robot.storage.setStorageMotor(0);
        Robot.shooter.setShooterMotor(0);
        Robot.shooter.resetPID();
    }
}
