// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.PDH.DEFAULT_BREAKER_ENTRIES;
import static frc.robot.Constants.Storage.RETRACT_MODE_FINISHED;
import static frc.robot.Constants.TestValue.DEFAULT_TEST_MODE;
import static frc.robot.robot_utils.TestUtil.TestMode.CHASSIS_DRIVE_TEST;
import static frc.robot.robot_utils.TestUtil.TestMode.CHASSIS_OFFSET_ADJUSTMENT;
import static frc.robot.robot_utils.TestUtil.TestMode.SHOOTER_ANGLE_TEST;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ChassisCameraConsts;
import frc.robot.Constants.ShooterCameraConsts;
import frc.robot.commands.shooter_commands.AutoAdjustShooterAngle;
import frc.robot.commands.test_commands.ChassisForwardOffsetTest;
import frc.robot.commands.test_commands.ChassisOffsetTest;
import frc.robot.commands.test_commands.ShooterAngleTest;
import frc.robot.robot_utils.ChassisCamera;
import frc.robot.robot_utils.ShooterCamera;
import frc.robot.robot_utils.TestUtil;
import frc.robot.robot_utils.power.BatteryManagement;
import frc.robot.subsystems.climber.LeftClimberSubsystem;
import frc.robot.subsystems.climber.RightClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.AngleAdjustSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.storage.AcceptColor;
import frc.robot.subsystems.storage.StorageSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import me.wobblyyyy.pathfinder2.Pathfinder;
import me.wobblyyyy.pathfinder2.wpilib.PathfinderSubsystem;

public class Robot extends TimedRobot {
    private Command autonomous;
    private static RobotContainer robotContainer;

    public static BatteryManagement bms = new BatteryManagement(DEFAULT_BREAKER_ENTRIES, 120);

    public static SwerveDriveSubsystem swerveDrive;
    public static Pathfinder pathfinder;
    public static PathfinderSubsystem pathfinderSubsystem;
    public static StorageSubsystem storage;
    public static ShooterSubsystem shooter;
    public static IntakeSubsystem intake;
    public static AngleAdjustSubsystem adjustor;
    public static TestUtil testUtil;

    public static LeftClimberSubsystem leftClimber;
    public static RightClimberSubsystem rightClimber;

    public HashMap<String, Double> targetData = new HashMap<>();

    public static final SendableChooser<AcceptColor> acceptColorChooser = new SendableChooser<>();
    public static final AcceptColor DEFAULT_COLOR = AcceptColor.NEUTRAL;

    public static ShooterCamera shooterCamera;
    public static ChassisCamera chassisCamera;

    public static boolean leftHandedMode = false;

    private AutoAdjustShooterAngle adjustAngle = new AutoAdjustShooterAngle();

    private void setupColorChooser() {
        // Add the values for the SendableChooser
        acceptColorChooser.addOption("Blue Accept", AcceptColor.BLUE);
        acceptColorChooser.addOption("Red Accept", AcceptColor.RED);
        acceptColorChooser.addOption("Neutral", AcceptColor.NEUTRAL);

        switch (DEFAULT_COLOR) {
            case RED:
                acceptColorChooser.setDefaultOption("Red Accept", AcceptColor.RED);
            case BLUE:
                acceptColorChooser.setDefaultOption("Blue Accept", AcceptColor.BLUE);
            case NEUTRAL:
                acceptColorChooser.setDefaultOption("Neutral", AcceptColor.NEUTRAL);
        }

        SmartDashboard.putData("Acceptance Color Chooser", acceptColorChooser);
    }

    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        
        swerveDrive = new SwerveDriveSubsystem();
        swerveDrive.resetGyro();
        pathfinder = swerveDrive.getPathfinder();
        pathfinderSubsystem = new PathfinderSubsystem(pathfinder);

        this.setupColorChooser();

        storage = new StorageSubsystem(acceptColorChooser::getSelected).setRetractMode(RETRACT_MODE_FINISHED);

        SmartDashboard.putBoolean("climber overheat", false);

        shooter = new ShooterSubsystem();
        adjustor = new AngleAdjustSubsystem();
        
        // updates the acceleration every 2 ms starting 1 ms after the robot starts
        addPeriodic(() -> { 
            
            if(!adjustAngle.isScheduled()){
                adjustAngle.schedule();
            }
        }, 0.2, 0.001);

        intake = new IntakeSubsystem();

        leftClimber = new LeftClimberSubsystem();
        rightClimber = new RightClimberSubsystem();

        // Add your test commands here
        testUtil = new TestUtil()
                .addDefaultCommand(CHASSIS_DRIVE_TEST, new ChassisForwardOffsetTest())
                .addDefaultCommand(CHASSIS_OFFSET_ADJUSTMENT, new ChassisOffsetTest())
                .addDefaultCommand(SHOOTER_ANGLE_TEST, new ShooterAngleTest())
                .setTestMode(DEFAULT_TEST_MODE);

        robotContainer = new RobotContainer();

        // Cameras
        chassisCamera = new ChassisCamera(
                "RoxBallCam",
                ChassisCameraConsts.CAMERA_HEIGHT,
                ChassisCameraConsts.CAMERA_PITCH,
                acceptColorChooser::getSelected
        );

        shooterCamera = new ShooterCamera(
                "RoxShooterCam",
                ShooterCameraConsts.CAMERA_HEIGHT,
                ShooterCameraConsts.CAMERA_PITCH
        );
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        bms.periodic();
        SmartDashboard.putNumber("PDP Voltage", bms.getVoltage());
        SmartDashboard.putNumber("PDP Total Amps", bms.getCurrent());
        SmartDashboard.putNumber("PDP Total Watts", bms.getWattage());
        SmartDashboard.putNumber("PDP Peak Current", bms.getMaximumCurrent());

        SmartDashboard.putNumber("PDP Watt-Hours", bms.getWattHours());
        SmartDashboard.putBoolean("PDP Exceeding Current", bms.isOverCurrentLimit());
    }

    @Override public void disabledInit() { CommandScheduler.getInstance().cancelAll(); }
    @Override public void disabledPeriodic() {}
    @Override public void autonomousPeriodic() {}

    @Override 
    public void teleopPeriodic() {
        targetData = Robot.shooterCamera.getTargetGoal();

        SmartDashboard.putNumber("Distance", targetData.get("Distance"));
        SmartDashboard.putNumber("Pitch", targetData.get("Pitch"));
        SmartDashboard.putNumber("Current Target Angle", RobotContainer.incrementAngleCMD.getCurrentTargetAngle());
    }


    @Override public void testPeriodic() {
    }

    @Override
    public void autonomousInit() {
        //autonomous = robotContainer.getAutonomousCommand(pathfinderSubsystem);

        CommandScheduler.getInstance().cancelAll();

        autonomous = robotContainer.getSimpleAutoCommand();
        Robot.adjustor.zero();
        
        if (autonomous != null)
            autonomous.schedule();
    }

    @Override
    public void teleopInit() {
        if (autonomous != null && !autonomous.isFinished())
            autonomous.cancel();

        CommandScheduler.getInstance().cancelAll();

        // TODO: Please comment this out when autonomous starts getting tested, it will mess up otherwise.
        Robot.adjustor.zero();
        robotContainer.resetIncrementAngle();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        new ShooterAngleTest().schedule();
    }
}
