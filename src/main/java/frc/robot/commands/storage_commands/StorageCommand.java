package frc.robot.commands.storage_commands;


import java.util.concurrent.TimeUnit;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.MotorValue;
import frc.robot.commands.intake_commands.EnableIntake;
import frc.robot.commands.intake_commands.SpinIntakeInward;
import frc.robot.robot_utils.MotorUtil;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.StorageSubsystem.Task;

public class StorageCommand extends CommandBase {
    
    private final StorageSubsystem storageSubsystem;
    private String currentState = "Idle";

    public StorageCommand(StorageSubsystem subsystem) {
        this.storageSubsystem = subsystem;

        addRequirements(storageSubsystem);
    }


    @Override
    public void initialize() {
        // The intake motor should always be running while the robot is active.
        storageSubsystem.addListener(new StorageSubsystem.StorageListener() {

            @Override
            public void colorFound(Task requiredTask, int ballsLoaded) {
                try {
                    switch (requiredTask) {
                        case ACCEPT: {
                            // Check how many balls there are already loaded
                            switch (ballsLoaded) {
                                case 0: {
                                    // There are zero balls loaded, run both motors until the back
                                    // sensor is tripped.
                                    storageSubsystem.setAcceptorMotor(MotorUtil.getMotorValue(MotorValue.ACCEPT_SPEED, MotorValue.ACCEPTOR_FLIPPED));
                                    storageSubsystem.setStorageMotor(MotorUtil.getMotorValue(MotorValue.SLOW_ACCEPT_SPEED, MotorValue.STORAGE_FLIPPED));

                                    TimeUnit.MILLISECONDS.sleep(100);

                                    while (!storageSubsystem.getStorageSensorCovered()) {
                                        CANSparkMax stalledMotor = MotorUtil.getStalledMotor();

                                        if (stalledMotor == storageSubsystem.getAcceptorMotor() || stalledMotor == storageSubsystem.getStorageMotor()) {
                                            break;
                                        }

                                        storageSubsystem.setAcceptorMotor(MotorUtil.getMotorValue(MotorValue.ACCEPT_SPEED, MotorValue.ACCEPTOR_FLIPPED));
                                        storageSubsystem.setStorageMotor(MotorUtil.getMotorValue(MotorValue.SLOW_ACCEPT_SPEED, MotorValue.STORAGE_FLIPPED));
                                    }
                                }
                                case 1: {
                                    // There is already a single ball loaded, run only the Acceptor
                                    // motor so the other ball is not pushed into the shooter area.
                                    storageSubsystem.setAcceptorMotor(MotorUtil.getMotorValue(MotorValue.ACCEPT_SPEED, MotorValue.ACCEPTOR_FLIPPED));
                                    TimeUnit.MILLISECONDS.sleep(100);

                                    CANSparkMax stalledMotor = MotorUtil.getStalledMotor();

                                    if (stalledMotor == storageSubsystem.getAcceptorMotor()) {
                                        break;
                                    }

                                    while (!storageSubsystem.getAcceptorSensorCovered() && StorageSubsystem.stalledMotor == null) {
                                        storageSubsystem.setAcceptorMotor(MotorUtil.getMotorValue(MotorValue.ACCEPT_SPEED, MotorValue.ACCEPTOR_FLIPPED));
                                    }

                                    TimeUnit.MILLISECONDS.sleep(250);
                                }
                            
                                // If there is more than one, do nothing because more cannot be accepted.
                                default: {}
                            }
                        }
                        case DENY: {
                            // This ball should be denied, spin the motor in the reject direction.
                            storageSubsystem.setAcceptorMotor(MotorUtil.getMotorValue(-MotorValue.ACCEPT_SPEED-0.1, MotorValue.ACCEPTOR_FLIPPED));
        
                            // Add a slight delay to the end to make sure the ball is thrown out.
                            TimeUnit.MILLISECONDS.sleep(250);
                        }
                    } 
                } catch (InterruptedException e) {}
                
                storageSubsystem.setAcceptorMotor(0);
                storageSubsystem.setStorageMotor(0);
            }

            @Override
            public void colorTimeoutError() {
                storageSubsystem.setAcceptorMotor(0);
                storageSubsystem.setStorageMotor(0);
            }
        });
    }
    
    @Override
    public void execute() {
        // TODO: Add general readings such as current, rpm, etc.
        SmartDashboard.putString("Storage State", currentState);

        // Run the motors responsible for extending the intake out of the robot, since 
        // these connect closely together.
        CommandScheduler.getInstance().schedule(new EnableIntake());

        // At the same time, start running the Intake motors at the intake direction, to move
        // the ball into the accept/reject Storage module, where decision can be made.
        CommandScheduler.getInstance().schedule(new SpinIntakeInward());

        // In the Dashboard, a Boolean block changes color based on true/false so it can
        // possibly be flashing if there is a problem that requires interaction.
        SmartDashboard.putBoolean("Storage Error", false);
    }

    @Override
    public void end(boolean interrupted) {
        // If the command ends, possibly turn off the motor?
        storageSubsystem.setStorageMotor(0);
        storageSubsystem.setAcceptorMotor(0);
    }

    @Override
    public boolean isFinished() {
        // Should run forever, so always return false.
        return false;
    }
}