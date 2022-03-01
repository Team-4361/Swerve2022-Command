package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

import static frc.robot.Constants.MotorFlip.ACCEPTOR_FLIPPED;
import static frc.robot.Constants.MotorFlip.SHOOTER_FLIPPED;
import static frc.robot.Constants.MotorValue.ACCEPT_SPEED;
import static frc.robot.Constants.MotorValue.SHOOT_SPEED;
import static frc.robot.Constants.Shooter.DESIRED_RPM;
import static frc.robot.robot_utils.MotorUtil.getMotorValue;

public class SensorShootCommand extends CommandBase {
    private double desiredRPM = DESIRED_RPM;

    /**
     * Initializes the Sensor Shooting Command, used to specify what your desired RPM is.
     *
     * @param desiredRPM The velocity you want to shoot at.
     */
    public SensorShootCommand(double desiredRPM) {
        this.desiredRPM = desiredRPM;

        addRequirements(Robot.shooter, Robot.storage);
    }

    /** Initializes the Sensor Shooting Command, with a default RPM from {@link Constants} */
    public SensorShootCommand() {
        addRequirements(Robot.shooter, Robot.storage);
    }

    

    /** Executes the Shooter Command */
    @Override
    public void execute() {
        // While the rear sensor is still being pressed keep executing the command (there is still a ball inside)
        if (Robot.storage.rearProximityCovered()) {
            // Constantly run the shooter, even while at the proper RPM.

            // If the desired RPM has been set, run the motor at the calculated velocity to make sure it
            // reaches the required speed.
            if (desiredRPM != DESIRED_RPM) {
                Robot.shooter.setShooterVelocity(desiredRPM);
            } else {
                Robot.shooter.setShooterMotor(SHOOT_SPEED);
            }

            // Only run the rear storage motor when the shooter has reached its target RPM.
            if (Robot.shooter.isDesiredSpeed(this.desiredRPM)) {
                // Run the storage motor, due to the if statement above it will shut off when the ball leaves.
                Robot.storage.setStorageMotor(ACCEPT_SPEED);
            }
        } else {
            end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Cut everything and shut off all the motors.
        Robot.storage.setStorageMotor(0);
        Robot.shooter.setShooterMotor(0);
    }
}
