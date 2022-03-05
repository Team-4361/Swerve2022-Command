package frc.robot.commands.shooter_commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.robot_utils.motor.MotorError;
import frc.robot.robot_utils.motor.MotorListener;
import frc.robot.robot_utils.motor.ProtectedAbstractMotor;
import me.wobblyyyy.pathfinder2.revrobotics.SparkMaxMotor;

public class CalibrateShooterCommand extends CommandBase {
    private boolean finished;
    private CANSparkMax sparkMotor;
    private SparkMaxMotor sparkMaxMotor;
    private RelativeEncoder sparkEncoder;

    public CalibrateShooterCommand() {
        this.finished = false;
        this.sparkMotor = Robot.adjustor.getAdjustor().getSpark();
        this.sparkMaxMotor = Robot.adjustor.getAdjustor();
        this.sparkEncoder = sparkMotor.getEncoder();
    }

    @Override
    public void initialize() {
        addRequirements(Robot.adjustor);
        finished = false;
    }

    @Override
    public void execute() {
        if (Math.abs(sparkEncoder.getVelocity()) < 50 && sparkMotor.getOutputCurrent() > 20) {
            // The motor has stalled, everything is done.
            finished = true;
            sparkMaxMotor.setPower(0);
        } else {
            sparkMaxMotor.setPower(-0.15);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // The motor is at the bottom position, end everything and zero out.
        sparkMaxMotor.setPower(0);
        Robot.adjustor.zero();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
