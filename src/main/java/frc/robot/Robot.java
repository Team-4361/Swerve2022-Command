// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ChassisCameraConsts;
import frc.robot.Constants.ShooterCameraConsts;
import frc.robot.commands.intake_commands.CalibrateIntake;
import frc.robot.commands.intake_commands.adjustor.ExtendIntake;
import frc.robot.commands.intake_commands.adjustor.ExtendIntakeMagnet;
import frc.robot.commands.test_commands.ChassisForwardOffsetTest;
import frc.robot.commands.test_commands.ChassisOffsetTest;
import frc.robot.commands.test_commands.IntakeMaxRotationTest;
import frc.robot.commands.test_commands.ShooterAngleTest;
import frc.robot.robot_utils.ChassisCamera;
import frc.robot.robot_utils.ShooterCamera;
import frc.robot.robot_utils.TestUtil;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.AngleAdjustSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.storage.AcceptColor;
import frc.robot.subsystems.storage.NewStorageSubsystem;

import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import me.wobblyyyy.pathfinder2.Pathfinder;
import me.wobblyyyy.pathfinder2.wpilib.PathfinderSubsystem;

import static frc.robot.Constants.Storage.RETRACT_MODE;
import static frc.robot.Constants.TestValue.DEFAULT_TEST_MODE;
import static frc.robot.robot_utils.TestUtil.TestMode.*;

public class Robot extends TimedRobot {
    private Command autonomous;

    private RobotContainer robotContainer;

    public static SwerveDriveSubsystem swerveDrive;
    public static Pathfinder pathfinder;
    public static PathfinderSubsystem pathfinderSubsystem;
    public static NewStorageSubsystem storage;
    public static ShooterSubsystem shooter;
    public static IntakeSubsystem intake;
    public static ClimberSubsystem climber;
    public static AngleAdjustSubsystem adjustor;
    public static TestUtil testUtil;

    public static ShooterCamera shooterCamera;
    public static ChassisCamera chassisCamera;
    public static boolean leftHandedMode = false;

    private final AcceptColor INIT_TARGET_COLOR = AcceptColor.BLUE;

    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        swerveDrive = new SwerveDriveSubsystem();
        swerveDrive.resetGyro();
        pathfinder = swerveDrive.getPathfinder();
        pathfinderSubsystem = new PathfinderSubsystem(pathfinder);

        storage = new NewStorageSubsystem(INIT_TARGET_COLOR)
                .setRetractMode(RETRACT_MODE);

        shooter = new ShooterSubsystem();

        intake = new IntakeSubsystem();
        climber = new ClimberSubsystem();

        adjustor = new AngleAdjustSubsystem();

        // Add your test commands here
        testUtil = new TestUtil()
                .addDefaultCommand(CHASSIS_DRIVE_TEST, new ChassisForwardOffsetTest())
                .addDefaultCommand(CHASSIS_OFFSET_ADJUSTMENT, new ChassisOffsetTest())
                .addDefaultCommand(SHOOTER_ANGLE_TEST, new ShooterAngleTest())
                .addDefaultCommand(INTAKE_ROTATION_TEST, new IntakeMaxRotationTest())
                .setTestMode(DEFAULT_TEST_MODE);

        robotContainer = new RobotContainer();

        //Cameras
        chassisCamera = new ChassisCamera("RoxBallCam", ChassisCameraConsts.CAMERA_HEIGHT, ChassisCameraConsts.CAMERA_PITCH, INIT_TARGET_COLOR);
        shooterCamera = new ShooterCamera("RoxShooterCam", ShooterCameraConsts.CAMERA_HEIGHT, ShooterCameraConsts.CAMERA_PITCH);

        // Run retract intake command (not sure if this is how it really works)
        //new CalibrateIntake().schedule();
    }


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }
    
    @Override public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        // autonomous = robotContainer
        //         .getAutonomousCommand(pathfinderSubsystem);

        if (autonomous != null)
            autonomous.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (autonomous != null && !autonomous.isFinished())
            autonomous.cancel();

        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        // Run the testing command when in testing mode.
        testUtil.runExecutedCommand();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
