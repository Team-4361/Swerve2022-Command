// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.LeftClimberSubsystem;
import frc.robot.subsystems.climber.RightClimberSubsystem;
import frc.robot.subsystems.intake.IntakeExtendSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.AngleAdjustSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.storage.StorageSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.power.BatteryManagement;

import static frc.robot.Constants.*;

/**
 * This {@link Robot} class is designed to contain all the timed methods, and many of the
 * Dynamic Global Variables that need to be defined <i>when the robot starts.</i>
 */
public class Robot extends TimedRobot {
    public static BatteryManagement bms;
    public static RobotContainer robotContainer;

    public static SwerveDriveSubsystem swerveDrive;
    public static StorageSubsystem storage;
    public static ShooterSubsystem shooter;
    public static IntakeSubsystem intake;
    public static IntakeExtendSubsystem intakeExtender;
    public static AngleAdjustSubsystem adjustor;

    public static LeftClimberSubsystem leftClimber;
    public static RightClimberSubsystem rightClimber;

    public boolean scheduled = false;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        bms = new BatteryManagement(Power.DEFAULT_BREAKER_ENTRIES, 120);
        swerveDrive = new SwerveDriveSubsystem();
        storage = new StorageSubsystem();
        shooter = new ShooterSubsystem();
        intake = new IntakeSubsystem();
        intakeExtender = new IntakeExtendSubsystem();
        adjustor = new AngleAdjustSubsystem();
        leftClimber = new LeftClimberSubsystem();
        rightClimber = new RightClimberSubsystem();
    
        adjustor.zero();

        robotContainer = new RobotContainer();
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();

        // reset the gyroscope
        swerveDrive.resetPosition();

        // TODO: add auto group to prevent errors.

        /*
        if (autoGroup != null && !scheduled) {
            autoGroup.schedule();
            scheduled = true;
        }

         */
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        bms.periodic();

        SmartDashboard.putNumber("PDP: Voltage", bms.getVoltage());
        SmartDashboard.putNumber("PDP: Total Amps", bms.getCurrent());
        SmartDashboard.putNumber("PDP: Total Watts", bms.getWattage());
        SmartDashboard.putNumber("PDP: Peak Current", bms.getMaximumCurrent());

        SmartDashboard.putNumber("PDP: Watt-Hours", bms.getWattHours());
        SmartDashboard.putBoolean("PDP: Exceeding Current", bms.isOverCurrentLimit());
    }

    @Override
    public void disabledInit() {
        scheduled = false;
    }

    @Override
    public void teleopInit() {
        // Cancel all commands that are currently running before starting teleop mode.
        CommandScheduler.getInstance().cancelAll();
        Robot.adjustor.zero();

        robotContainer.getArcadeDriveCommand().schedule();
    }
}
