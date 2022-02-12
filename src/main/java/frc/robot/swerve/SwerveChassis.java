package frc.robot.swerve;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import me.wobblyyyy.pathfinder2.geometry.Translation;
import me.wobblyyyy.pathfinder2.robot.Drive;

public class SwerveChassis implements Drive {
    private static final Translation2d SWERVE_FR_POSITION =
            new Translation2d(Chassis.SWERVE_CHASSIS_SIDE_LENGTH/2, Chassis.SWERVE_CHASSIS_SIDE_LENGTH/2);
    private static final Translation2d SWERVE_FL_POSITION =
            new Translation2d(-Chassis.SWERVE_CHASSIS_SIDE_LENGTH/2, Chassis.SWERVE_CHASSIS_SIDE_LENGTH/2);
    private static final Translation2d SWERVE_BR_POSITION =
            new Translation2d(Chassis.SWERVE_CHASSIS_SIDE_LENGTH/2, -Chassis.SWERVE_CHASSIS_SIDE_LENGTH/2);
    private static final Translation2d SWERVE_BL_POSITION =
            new Translation2d(-Chassis.SWERVE_CHASSIS_SIDE_LENGTH/2, -Chassis.SWERVE_CHASSIS_SIDE_LENGTH/2);
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
                new SwerveModule(Chassis.FR_DRIVE_ID, Chassis.FR_TURN_ID, Chassis.FR_DIO_ENCODER_PORT, Chassis.FR_OFFSET),
                new SwerveModule(Chassis.FL_DRIVE_ID, Chassis.FL_TURN_ID, Chassis.FL_DIO_ENCODER_PORT, Chassis.FL_OFFSET),
                new SwerveModule(Chassis.BR_DRIVE_ID, Chassis.BR_TURN_ID, Chassis.BR_DIO_ENCODER_PORT, Chassis.BR_OFFSET),
                new SwerveModule(Chassis.BL_DRIVE_ID, Chassis.BL_TURN_ID, Chassis.BL_DIO_ENCODER_PORT, Chassis.BL_OFFSET)
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

    public HashMap<String, SwerveModuleState> getSwerveModuleStates(){
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

        frontRight.setState(frontRightState);
        frontLeft.setState(frontLeftState);
        backRight.setState(backRightState);
        backLeft.setState(backLeftState);

        updateDashboard();
    }

    @Override
    public void setModifier(Function<Translation, Translation> modifier) {
        this.modifier = modifier;
    }

    @Override
    public Function<Translation, Translation> getModifier() {
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
}
