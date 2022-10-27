package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.function.Supplier;

import static frc.robot.Constants.Chassis.DRIVE_DEAD_ZONE;
//import static frc.robot.commands.chassis.werveDriveMode.FIELD_RELATIVE;
import static frc.robot.utils.motor.MotorUtil.deadzone;

/**
 * This {@link ArcadeDriveCommand} is the main {@link Command} designed for driving the Robot during
 * the teleoperated mode. By default, this reads the input from two Joysticks, one for direction, and one for
 * rotation.
 * <p></p>
 * We are now experimenting by using a Gyroscope to offset the Robot for field-relative control, which is the
 * method used by default. During operation, it should be able to be switched to robot-relative control, which will
 * internally freeze the Gyroscope readings in its <b>last-read position</b>, in order to not switch the head direction
 * while its being driven.
 * <p></p>
 * Based on the Gyroscopic readings of the Robot, the
 * {@link ChassisSpeeds#fromFieldRelativeSpeeds(double, double, double, Rotation2d)} should be able to convert the
 * field-relative speeds with the {@link Rotation2d} gyro angle into robot-relative speeds that the system should
 * be able to handle, without significantly modifying how our {@link SwerveDriveSubsystem} will work, with the
 * ability to switch to relative, if it is ever desired.
 * <p></p>
 * <a href="https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html">
 *     Intro to ChassisSpeeds and Field-Relative Swerve Drive.</a>
 */
public class ArcadeDriveCommand extends CommandBase {
    private final Supplier<Double> xSupplier, ySupplier, twistSupplier;

    /** Creates a new {@link ArcadeDriveCommand} using the default variable {@link Supplier}*/
    public ArcadeDriveCommand(Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> twistSupplier) {
        addRequirements(Robot.swerveDrive);

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.twistSupplier = twistSupplier;
    }

    public Supplier<Double> getXSupplier() {
        return this.xSupplier;
    }

    public Supplier<Double> getYSupplier() {
        return this.ySupplier;
    }

    public Supplier<Double> getTwistSupplier() {
        return this.twistSupplier;
    }

    @Override
    public void execute() {
        Robot.swerveDrive.autoDrive(
                    deadzone(xSupplier.get(), DRIVE_DEAD_ZONE),
                    -deadzone(ySupplier.get(), DRIVE_DEAD_ZONE),
                    -deadzone(twistSupplier.get(), DRIVE_DEAD_ZONE)
        );
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
