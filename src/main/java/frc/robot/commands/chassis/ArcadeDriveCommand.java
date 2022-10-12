package frc.robot.commands.chassis;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.motor.MotorUtil;

import java.util.function.Supplier;

import static frc.robot.Constants.Chassis.DRIVE_DEAD_ZONE;
import static frc.robot.Constants.MotorFlip.GYRO_FLIPPED;
import static frc.robot.commands.chassis.SwerveDriveMode.FIELD_RELATIVE;
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
    private Rotation2d gyroAngle;

    private final Supplier<Double> xSupplier, ySupplier, twistSupplier;
    private boolean gyroResetOnInit = true;

    // used for testing purposes, holds the value of the joystick without requiring constant input.
    private boolean holdMode = false;
    private double xVal=0, yVal=0, twistVal=0;

    /** Creates a new {@link ArcadeDriveCommand} using the default variable {@link Supplier}*/
    public ArcadeDriveCommand(Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> twistSupplier) {
        addRequirements(Robot.swerveDrive);

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.twistSupplier = twistSupplier;
        this.gyroAngle = new Rotation2d(0);
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

    public boolean isGyroResetOnInit() {
        return this.gyroResetOnInit;
    }

    public ArcadeDriveCommand setGyroResetOnInit(boolean val) {
        this.gyroResetOnInit = val;
        return this;
    }

    public void setHoldMode(boolean holdMode) {
        this.holdMode = holdMode;
    }

    public boolean getHoldMode() {
        return this.holdMode;
    }

    @Override
    public void execute() {
        if (Robot.swerveDrive.getDriveMode() == FIELD_RELATIVE) {
            // Update the gyro angle to allow field-relative control.
            gyroAngle = Robot.swerveDrive.getGyro();

            if (GYRO_FLIPPED) {
                gyroAngle = gyroAngle.unaryMinus();
            }
        }

        if (!holdMode) {
            this.xVal = xSupplier.get();
            this.yVal = ySupplier.get();
            this.twistVal = twistSupplier.get();
        }

        Robot.swerveDrive.drive(
                // This converts the inputs to a robot-relative control that can be processed by the SwerveDriveSubsystem
                // and with the updated gyroAngle it can either be field-relative or robot-relative.
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    deadzone(xVal, DRIVE_DEAD_ZONE),
                    -deadzone(yVal, DRIVE_DEAD_ZONE),
                    -deadzone(twistVal, DRIVE_DEAD_ZONE),
                    gyroAngle
                )
        );
    }

    @Override
    public void initialize() {
        if (gyroResetOnInit) {
            Robot.swerveDrive.resetGyro();
        }
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
