package frc.robot.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import me.wobblyyyy.pathfinder2.geometry.Translation;
import me.wobblyyyy.pathfinder2.robot.Drive;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.TestValue.DRIVE_ENABLED;

@SuppressWarnings("unused")
public class SwerveChassis implements Drive {
    private static final Translation2d SWERVE_FR_POSITION =
            new Translation2d(SWERVE_CHASSIS_SIDE_LENGTH / 2, SWERVE_CHASSIS_SIDE_LENGTH / 2);
    private static final Translation2d SWERVE_FL_POSITION =
            new Translation2d(-SWERVE_CHASSIS_SIDE_LENGTH / 2, SWERVE_CHASSIS_SIDE_LENGTH / 2);
    private static final Translation2d SWERVE_BR_POSITION =
            new Translation2d(SWERVE_CHASSIS_SIDE_LENGTH / 2, -SWERVE_CHASSIS_SIDE_LENGTH / 2);
    private static final Translation2d SWERVE_BL_POSITION =
            new Translation2d(-SWERVE_CHASSIS_SIDE_LENGTH / 2, -SWERVE_CHASSIS_SIDE_LENGTH / 2);
    private static final SwerveDriveKinematics SWERVE_KINEMATICS =
            new SwerveDriveKinematics(
                    SWERVE_FR_POSITION,
                    SWERVE_FL_POSITION,
                    SWERVE_BR_POSITION,
                    SWERVE_BL_POSITION
            );

    private static final String NAME_FR = "FR";
    private static final String NAME_FL = "FL";
    private static final String NAME_BR = "BR";
    private static final String NAME_BL = "BL";

    private final SwerveModule frontRight;
    private final SwerveModule frontLeft;
    private final SwerveModule backRight;
    private final SwerveModule backLeft;

    private Function<Translation, Translation> modifier = (t) -> t;
    private Translation translation;

    public SwerveChassis() {
        this(
                new SwerveModule(
                        FR_DRIVE_ID,
                        FR_TURN_ID,
                        FR_DIO_ENCODER_PORT,
                        FR_OFFSET,
                        FR_ERROR_FACTOR
                ),
                new SwerveModule(
                        FL_DRIVE_ID,
                        FL_TURN_ID,
                        FL_DIO_ENCODER_PORT,
                        FL_OFFSET,
                        FL_ERROR_FACTOR
                ),
                new SwerveModule(
                        BR_DRIVE_ID,
                        BR_TURN_ID,
                        BR_DIO_ENCODER_PORT,
                        BR_OFFSET,
                        BR_ERROR_FACTOR
                ),
                new SwerveModule(
                        BL_DRIVE_ID,
                        BL_TURN_ID,
                        BL_DIO_ENCODER_PORT,
                        BL_OFFSET,
                        BL_ERROR_FACTOR
                )
        );
    }

    public SwerveChassis(SwerveModule frontRight,
                         SwerveModule frontLeft,
                         SwerveModule backRight,
                         SwerveModule backLeft) {
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;
    }

    private void updateDashboard() {
        frontRight.updateDashboard(NAME_FR);
        frontLeft.updateDashboard(NAME_FL);
        backRight.updateDashboard(NAME_BR);
        backLeft.updateDashboard(NAME_BL);
    }

    public SwerveModule getFrontRight() {
        return frontRight;
    }

    public SwerveModule getFrontLeft() {
        return frontLeft;
    }

    public SwerveModule getBackRight() {
        return backRight;
    }

    public SwerveModule getBackLeft() {
        return backLeft;
    }

    public SwerveDriveKinematics getSwerveKinematics() {
        return SWERVE_KINEMATICS;
    }

    public HashMap<String, SwerveModuleState> getSwerveModuleStates() {
        return new HashMap<>(Map.of("FL", getFrontLeft().getState(),
                "BL", getBackLeft().getState(),
                "FR", getFrontRight().getState(),
                "BR", getBackRight().getState()));
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states =
                SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

        SwerveModuleState frontRightState = states[0];
        SwerveModuleState frontLeftState = states[1];
        SwerveModuleState backRightState = states[2];
        SwerveModuleState backLeftState = states[3];

        if (DRIVE_ENABLED) {
            frontRight.setState(frontRightState);
            frontLeft.setState(frontLeftState);
            backRight.setState(backRightState);
            backLeft.setState(backLeftState);
        }

        // don't update the dashboard here anymore, do it in SwerveOdometry
        // updateDashboard();
    }

    @Override
    public void setDriveModifier(Function<Translation, Translation> modifier) {
        this.modifier = modifier;
    }

    @Override
    public Function<Translation, Translation> getDriveModifier() {
        return modifier;
    }

    @Override
    public void setTranslation(Translation translation) {
        this.translation = translation;
    }

    @Override
    public Translation getTranslation() {
        return translation;
    }

    public double getDistance() {
        return frontRight.getDistance() * 2 *
            Math.PI * Constants.Chassis.SWERVE_WHEEL_RADIUS;
    }

    public void resetDriveEncoders() {
        frontRight.resetDriveEncoder();
    }

    public SwerveChassis getSwerveChassis(){
        return this;
    }
}
