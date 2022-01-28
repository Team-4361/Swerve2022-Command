package frc.robot.commands;

import static frc.robot.Constants.INTAKE_MOTOR_SPEED;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.AcceptColor;

public class IntakeCommand extends CommandBase {
    
    private final IntakeSubsystem intakeSubsystem;

    private boolean proximityValue;
    private AcceptColor acceptColor = IntakeSubsystem.AcceptColor.BLUE;
    private Color color;

    private double blueThreshold = 0.35;
    private double redThreshold = 0.35;

    public IntakeCommand(IntakeSubsystem subsystem, IntakeSubsystem.AcceptColor acceptColor) {
        this.intakeSubsystem = subsystem;
        this.acceptColor = acceptColor;

        addRequirements(intakeSubsystem);
    }

    /** Updates the Color and Proximity Values */
    public void updateValues() {
        this.color = intakeSubsystem.getColorValue();
        this.proximityValue = intakeSubsystem.getPhotoElectricProximity();
    }

    /**
     * Changes the Accept Color to Red or Blue
     * @param color Color
     */
    public void setAcceptColor(AcceptColor color) {
        this.acceptColor = color;
    }

    @Override
    public void initialize() {
        // The intake motor should always be running while the robot is active.
        intakeSubsystem.startIntakeMotor(INTAKE_MOTOR_SPEED);
    }

    
    @Override
    public void execute() {
        updateValues();

        // If the proximity sensor detects the ball is coming, attempt to check the color, and
        // when the reading is mostly correct, start ramping up the acceptor motor.
        if (AcceptColor.BLUE == acceptColor) {
            // Add Logic Here
        }
        else {

        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        // Should run forever, so always return false.
        return false;
    }
}