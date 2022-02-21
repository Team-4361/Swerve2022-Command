// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.TestValue.DEFAULT_TEST_MODE;
import static frc.robot.robot_utils.TestUtil.TestMode.CHASSIS_DRIVE_TEST;
import static frc.robot.robot_utils.TestUtil.TestMode.CHASSIS_OFFSET_ADJUSTMENT;
import static frc.robot.robot_utils.TestUtil.TestMode.SHOOTER_ANGLE_TEST;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.test_commands.ChassisDriveTest;
import frc.robot.commands.test_commands.ChassisOffsetTest;
import frc.robot.commands.test_commands.ShooterAngleTest;
import frc.robot.robot_utils.ChassisCamera;
import frc.robot.robot_utils.ShooterCamera;
import frc.robot.robot_utils.TestUtil;
import frc.robot.subsystems.*;
import me.wobblyyyy.pathfinder2.Pathfinder;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    public static SwerveDriveSubsystem swerveDrive;
    public static Pathfinder pathfinder;
    public static StorageSubsystem storage;
    public static ShooterSubsystem shooter;
    public static IntakeSubsystem intake;
    public static ClimberSubsystem climber;
    public static AngleAdjustSubsystem adjustor;
    public static TestUtil testUtil;

    public static ShooterCamera shooterCamera;
    public static ChassisCamera chassisCamera;
    public static boolean leftHandedMode = false;

    private final StorageSubsystem.AcceptColor INIT_TARGET_COLOR = StorageSubsystem.AcceptColor.BLUE;

    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.

        swerveDrive = new SwerveDriveSubsystem();
        swerveDrive.resetGyro();
        pathfinder = swerveDrive.getPathfinder();

        storage = new StorageSubsystem(INIT_TARGET_COLOR);
        shooter = new ShooterSubsystem(storage);

        intake = new IntakeSubsystem();
        climber = new ClimberSubsystem();

        adjustor = new AngleAdjustSubsystem();

        // Add your test commands here
        testUtil = new TestUtil()
                .addDefaultCommand(CHASSIS_DRIVE_TEST, new ChassisDriveTest())
                .addDefaultCommand(CHASSIS_OFFSET_ADJUSTMENT, new ChassisOffsetTest())
                .addDefaultCommand(SHOOTER_ANGLE_TEST, new ShooterAngleTest())
                .setTestMode(DEFAULT_TEST_MODE);


        //Should be the last thing in this function
        m_robotContainer = new RobotContainer();
    }


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}


    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand(pathfinder);

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        // Run the testing command when in testing mode.
        testUtil.runExecutedCommand();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
