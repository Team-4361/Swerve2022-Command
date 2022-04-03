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

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.intake.ExtenderSubsystem;
import org.w3c.dom.css.RGBColor;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ChassisCameraConsts;
import frc.robot.Constants.ShooterCameraConsts;
import frc.robot.commands.shooter_commands.AutoAdjustShooterAngle;
import frc.robot.commands.shooter_commands.SetShooterAngleCommand;
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
import frc.robot.subsystems.storage.RetractMode;
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
    public static ExtenderSubsystem intakeExtender;
    public static AngleAdjustSubsystem adjustor;
    public static TestUtil testUtil;

    public static LeftClimberSubsystem leftClimber;
    public static RightClimberSubsystem rightClimber;

    public HashMap<String, Double> targetData = new HashMap<>();

    public static final SendableChooser<AcceptColor> acceptColorChooser = new SendableChooser<>();
    public static final SendableChooser<AcceptColor> autonTargetBallChooser = new SendableChooser<>();

    public static final AcceptColor DEFAULT_COLOR = AcceptColor.NEUTRAL;

    public static final AcceptColor DEFAULT_AUTO_BALL = AcceptColor.BLUE;

    public static ShooterCamera shooterCamera;
    public static ChassisCamera chassisCamera;

    public static boolean leftHandedMode = false;

    private AutoAdjustShooterAngle adjustAngle;

    //public static SetShooterAngleCommand fixSetShooterAngle;

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

        autonTargetBallChooser.addOption("Auto Red", AcceptColor.RED);
        
        autonTargetBallChooser.addOption("Auto Ball", AcceptColor.BLUE);
        switch (DEFAULT_AUTO_BALL) {
            case RED:
                autonTargetBallChooser.setDefaultOption("Red Accept", AcceptColor.RED);
            case BLUE:
                autonTargetBallChooser.setDefaultOption("Blue Accept", AcceptColor.BLUE);
            case NEUTRAL:
                break;
        }

        SmartDashboard.putData("Autonomous Color Chooser", autonTargetBallChooser);
    }

    @Override
    public void disabledExit() {
        adjustor.zero();
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

        storage = new StorageSubsystem(acceptColorChooser::getSelected).setRetractMode(RetractMode.RETRACT_ALWAYS);

        SmartDashboard.putBoolean("climber overheat", false);

        shooter = new ShooterSubsystem();
        adjustor = new AngleAdjustSubsystem();

        adjustAngle = new AutoAdjustShooterAngle();
        
        // updates the acceleration every 2 ms starting 1 ms after the robot starts
        addPeriodic(() -> { 
            
            if(!adjustAngle.isScheduled()){
                adjustAngle.schedule();
            }
        }, 0.2, 0.001);

        intake = new IntakeSubsystem();
        intakeExtender = new ExtenderSubsystem();

        leftClimber = new LeftClimberSubsystem();
        rightClimber = new RightClimberSubsystem();

        leftClimber.zero();;
        rightClimber.zero();

        adjustor.zero();

        //fixSetShooterAngle = new SetShooterAngleCommand(10);

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
                autonTargetBallChooser::getSelected
        );

        shooterCamera = new ShooterCamera(
                "RoxShooterCam",
                ShooterCameraConsts.CAMERA_HEIGHT,
                ShooterCameraConsts.CAMERA_PITCH
        );
    }

    private double maxMemory = 0;

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        bms.periodic();
        SmartDashboard.putNumber("PDP Voltage", bms.getVoltage());
        SmartDashboard.putNumber("PDP Total Amps", bms.getCurrent());
        SmartDashboard.putNumber("PDP Total Watts", bms.getWattage());
        SmartDashboard.putNumber("PDP Peak Current", bms.getMaximumCurrent());

        SmartDashboard.putNumber("PDP Amp Hours", bms.getAmpHours());
        SmartDashboard.putNumber("PDP Watt-Hours", bms.getWattHours());
        SmartDashboard.putBoolean("PDP Exceeding Current", bms.isOverCurrentLimit());

        Runtime runtime = Runtime.getRuntime();
        long totalMemory = runtime.maxMemory();
        long freeMemory = runtime.freeMemory();

        long usedMemory = totalMemory - freeMemory;

        SmartDashboard.putNumber("Used Memory", usedMemory);
        SmartDashboard.putNumber("Free Memory", freeMemory);
        SmartDashboard.putNumber("Memory %", (usedMemory/536870912)*100);
    }

    @Override public void disabledInit() { CommandScheduler.getInstance().cancelAll(); }
    @Override public void disabledPeriodic() {}
    @Override public void autonomousPeriodic() {}

    private void getRequiring(Subsystem subsystem, String name) {
        Command reqCommand = CommandScheduler.getInstance().requiring(subsystem);

        if (reqCommand == null) {
            SmartDashboard.putString("requring " + name, "none");
        } else {
            SmartDashboard.putString("requring " + name, reqCommand.getName());
        }
    }

    @Override 
    public void teleopPeriodic() {
        targetData = Robot.shooterCamera.getTargetGoal();

        SmartDashboard.putNumber("Distance", targetData.get("Distance"));
        SmartDashboard.putNumber("Pitch", targetData.get("Pitch"));
        SmartDashboard.putNumber("Current Target Angle", RobotContainer.incrementAngleCMD.getCurrentTargetAngle());
   
        Command requringIntake = CommandScheduler.getInstance().requiring(Robot.intake);


        getRequiring(Robot.intake, "Intake");
        getRequiring(Robot.intakeExtender, "Intake Extender");
        getRequiring(Robot.storage, "Storage");
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
        if(autonomous != null)
            autonomous.cancel();

        CommandScheduler.getInstance().cancelAll();
        maxMemory = 0;
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        new ShooterAngleTest().schedule();
    }
}
