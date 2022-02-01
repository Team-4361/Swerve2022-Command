package frc.robot.commands;

import static frc.robot.Constants.INTAKE_MOTOR_SPEED;
import static frc.robot.Constants.ACCEPTOR_MOTOR_SPEED;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.StorageSubsystem.AcceptColor;
import frc.robot.subsystems.StorageSubsystem.Task;

public class StorageCommand extends CommandBase {
    
    private final StorageSubsystem storageSubsystem;

    private boolean proximityValue;
    private AcceptColor acceptColor = StorageSubsystem.AcceptColor.BLUE;
    private String currentState = "Idle";
    private Color color;

    public StorageCommand(StorageSubsystem subsystem, StorageSubsystem.AcceptColor acceptColor) {
        this.storageSubsystem = subsystem;
        this.acceptColor = acceptColor;

        addRequirements(storageSubsystem);
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
        storageSubsystem.startIntakeMotor(INTAKE_MOTOR_SPEED);
        storageSubsystem.addListener(new StorageSubsystem.StorageListener() {

            @Override
            public void colorFound(Task requiredTask) {
                switch (requiredTask) {
                    case ACCEPT: {
                        // This ball should be accepted, spin the motor in the accept direction.
                        // TODO: Check motor direction
                        storageSubsystem.startAcceptorMotor(ACCEPTOR_MOTOR_SPEED);
                    }
                    case DENY: {
                        // This ball should be denied, spin the motor in the reject direction.
                        storageSubsystem.startAcceptorMotor(-ACCEPTOR_MOTOR_SPEED);
                    }
                }
            }

            @Override
            public void colorTimeoutError() {
                storageSubsystem.stopAcceptorMotor();
            }
        });
    }

    
    @Override
    public void execute() {
        // TODO: Add general readings such as current, rpm, etc.
        SmartDashboard.putString("Storage State", currentState);

        // In the Dashboard, a Boolean block changes color based on true/false so it can
        // possibly be flashing if there is a problem that requires interaction.
        SmartDashboard.putBoolean("Storage Error", false);
    }

    @Override
    public void end(boolean interrupted) {
        // If the command ends, possibly turn off the motor?
        storageSubsystem.stopIntakeMotor();
        storageSubsystem.stopAcceptorMotor();
    }

    @Override
    public boolean isFinished() {
        // Should run forever, so always return false.
        return false;
    }
}