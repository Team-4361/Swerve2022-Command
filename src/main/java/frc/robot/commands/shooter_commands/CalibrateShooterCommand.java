package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.robot_utils.motor.MotorError;
import frc.robot.robot_utils.motor.MotorListener;
import frc.robot.robot_utils.motor.ProtectedAbstractMotor;

public class CalibrateShooterCommand extends CommandBase {
    private final ProtectedAbstractMotor protectedMotor;
    private boolean finished = false;

    public CalibrateShooterCommand() {
        this.protectedMotor = Robot.adjustor.getAdjustor();
    }

    @Override
    public void initialize() {
        addRequirements(Robot.adjustor);

        this.protectedMotor.setMaximumRunningCurrent(10);
        this.protectedMotor.setMaxTotalCurrent(30);

        this.protectedMotor.addListener(error -> {
            if (error.equals(MotorError.OVER_CURRENT)) {
                // The motor is stalling, so the adjustor is done
                // definitely not the best way, but it will work until
                // a limit switch is properly installed.
                this.finished = true;
            }
        });
    }

    @Override
    public void execute() {
        this.protectedMotor.setPower(-0.15);
    }

    @Override
    public void end(boolean interrupted) {
        // The motor is at the bottom position, end everything and zero out.
        this.protectedMotor.setPower(0);
        Robot.adjustor.zero();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
