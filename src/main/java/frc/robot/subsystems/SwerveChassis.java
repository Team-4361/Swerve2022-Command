package frc.robot.subsystems;


import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveChassis {
    private static final Translation2d SWERVE_FR_POSITION =
            new Translation2d(Constants.SWERVE_CHASSIS_SIDE_LENGTH/2, Constants.SWERVE_CHASSIS_SIDE_LENGTH/2);
    private static final Translation2d SWERVE_FL_POSITION =
            new Translation2d(-Constants.SWERVE_CHASSIS_SIDE_LENGTH/2, Constants.SWERVE_CHASSIS_SIDE_LENGTH/2);
    private static final Translation2d SWERVE_BR_POSITION =
            new Translation2d(Constants.SWERVE_CHASSIS_SIDE_LENGTH/2, -Constants.SWERVE_CHASSIS_SIDE_LENGTH/2);
    private static final Translation2d SWERVE_BL_POSITION =
            new Translation2d(-Constants.SWERVE_CHASSIS_SIDE_LENGTH/2, -Constants.SWERVE_CHASSIS_SIDE_LENGTH/2);
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



    public SwerveChassis() {
        this(
                new SwerveModule(Constants.FR_DRIVE_ID, Constants.FR_TURN_ID, Constants.FR_DIO_ENCODER_PORT, Constants.FR_OFFSET),
                new SwerveModule(Constants.FL_DRIVE_ID, Constants.FL_TURN_ID, Constants.FL_DIO_ENCODER_PORT, Constants.FL_OFFSET),
                new SwerveModule(Constants.BR_DRIVE_ID, Constants.BR_TURN_ID, Constants.BR_DIO_ENCODER_PORT, Constants.BR_OFFSET),
                new SwerveModule(Constants.BL_DRIVE_ID, Constants.BL_TURN_ID, Constants.BL_DIO_ENCODER_PORT, Constants.BL_OFFSET)
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
}
