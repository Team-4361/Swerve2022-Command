// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.chassis.ArcadeDriveCommand;
import frc.robot.commands.climber.LeftClimberDownCommand;
import frc.robot.commands.climber.LeftClimberUpCommand;
import frc.robot.commands.climber.RightClimberDownCommand;
import frc.robot.commands.climber.RightClimberUpCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.commands.shooter.DecreaseAngleCommand;
import frc.robot.commands.shooter.IncreaseAngleCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.storage.ProcessBallCommand;
import frc.robot.commands.storage.RunStorageAcceptor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.trigger.ConditionalButton;

import java.util.concurrent.atomic.AtomicInteger;

import static frc.robot.Constants.Control.*;
import static frc.robot.Constants.Control.XBOX_START;

/**
 * This {@link RobotContainer} class is designed to be used for Button Bindings, and the Command Groups
 * that are used to link multiple {@link Command}'s together.
 * <p>
 *     Since we are using the simplified robot code, many aspects of this {@link RobotContainer} has been removed,
 *     such as the autonomous and camera features. Some of the commands have also been changed in favor of
 *     simpler versions, and other changes have been made as well.
 * </p>
 *
 * @since 0.0.0
 */
public class RobotContainer {
    private final Joystick xyStick = new Joystick(XY_STICK_ID);
    private final Joystick zStick = new Joystick(Z_STICK_ID);
    public static final XboxController controller = new XboxController(CONTROLLER_ID);

    // Xbox Extracted Controller Buttons
    private final JoystickButton xButton = new JoystickButton(controller, XBOX_X);
    private final JoystickButton yButton = new JoystickButton(controller, XBOX_Y);
    private final JoystickButton aButton = new JoystickButton(controller, XBOX_A);
    private final JoystickButton bButton = new JoystickButton(controller, XBOX_B);
    private final JoystickButton lBumper = new JoystickButton(controller, XBOX_LEFT_TRIGGER);
    private final JoystickButton rBumper = new JoystickButton(controller, XBOX_RIGHT_TRIGGER);
    private final JoystickButton lStick = new JoystickButton(controller, XBOX_LEFT_STICK);
    private final JoystickButton rStick = new JoystickButton(controller, XBOX_RIGHT_STICK);
    private final JoystickButton startButton = new JoystickButton(controller, XBOX_START);

    private final ConditionalButton lTrigger = new ConditionalButton(controller, 100);
    private final ConditionalButton rTrigger = new ConditionalButton(controller, 101);
    private final ConditionalButton dpadDown = new ConditionalButton(controller, 102);
    private final ConditionalButton dpadUp =  new ConditionalButton(controller, 103);

    /** This shoot RPM will be able to be adjusted from the Joystick or {@link SmartDashboard}. */
    public AtomicInteger shootRPM = new AtomicInteger(4500);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        Robot.swerveDrive.setDefaultCommand(new ArcadeDriveCommand(xyStick::getX, xyStick::getY, zStick::getTwist));

        lTrigger.setSupplier(() -> (controller.getLeftTriggerAxis() > 0.8));
        rTrigger.setSupplier(() -> (controller.getRightTriggerAxis() > 0.8));
        dpadDown.setSupplier(() -> (controller.getPOV() == 180));
        dpadUp.setSupplier(() -> (controller.getPOV() == 0));
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        aButton.whenHeld(new ShootCommand());
        yButton.whenHeld(new ProcessBallCommand());
        xButton.whenActive(new RetractIntakeCommand());
        lTrigger.whenHeld(new LeftClimberDownCommand());
        rTrigger.whenHeld(new RightClimberDownCommand());
        lBumper.whenHeld(new LeftClimberUpCommand());
        rBumper.whenHeld(new RightClimberUpCommand());
        dpadUp.whenHeld(new IncreaseAngleCommand());
        dpadDown.whenHeld(new DecreaseAngleCommand());

        startButton.whenPressed(new InstantCommand(() -> {
            // Ensure the range is correct when adding, if this number is too high then the motor will never
            // reach its target. Since it's only allowed to increment if the number is 5k or below, it will only
            // obtain a maximum of 5500 RPM.
            if (shootRPM.get() <= 5000) {
                shootRPM.set(shootRPM.get()+500);
            }
        }));

        bButton.whenPressed(new InstantCommand(() -> {
            // Ensure the range is correct when subtracting, if this number is too low then the PIDController will
            // not give the motor enough power to spin up and throw a ball. Since it's only allowed to decrease if
            // the number is 1k rpm or above, it will only obtain a minimum of 500.
            if (shootRPM.get() >= 1000) {
                shootRPM.set(shootRPM.get()-500);
            }
        }));

        lStick.whenHeld(new RunStorageAcceptor());
    }
}
