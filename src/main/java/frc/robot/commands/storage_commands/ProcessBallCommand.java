package frc.robot.commands.storage_commands;


import java.util.concurrent.TimeUnit;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake_commands.DisableIntake;
import frc.robot.commands.intake_commands.EnableIntake;
import frc.robot.commands.intake_commands.SpinIntakeInward;
import frc.robot.commands.intake_commands.SpinIntakeOutward;
import frc.robot.robot_utils.MotorUtil;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.StorageSubsystem.Task;

import static frc.robot.Constants.*;

public class ProcessBallCommand extends CommandBase {
    
    private final StorageSubsystem storageSubsystem;
    private final ParallelCommandGroup enableGroup, disableGroup;

    private boolean finished = false;

    private final CANSparkMax[] processMotorGroup;

    public ProcessBallCommand(StorageSubsystem subsystem) {
        this.storageSubsystem = subsystem;

        enableGroup = new ParallelCommandGroup()
                .alongWith(new SpinIntakeOutward())
                .alongWith(new EnableIntake());

        disableGroup = new ParallelCommandGroup()
                .alongWith(new SpinIntakeInward())
                .alongWith(new DisableIntake());

        processMotorGroup = new CANSparkMax[]{storageSubsystem.getStorageMotor(), storageSubsystem.getAcceptorMotor()};

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
                                    storageSubsystem.setAcceptorMotor(MotorUtil.getMotorValue(MotorValue.ACCEPT_SPEED, MotorFlip.ACCEPTOR_FLIPPED));
                                    storageSubsystem.setStorageMotor(MotorUtil.getMotorValue(MotorValue.SLOW_ACCEPT_SPEED, MotorFlip.STORAGE_FLIPPED));

                                    TimeUnit.MILLISECONDS.sleep(100);

                                    while (!storageSubsystem.getStorageSensorCovered() && !MotorUtil.isAnyStalled(processMotorGroup)) {
                                        storageSubsystem.setAcceptorMotor(MotorUtil.getMotorValue(MotorValue.ACCEPT_SPEED, MotorFlip.ACCEPTOR_FLIPPED));
                                        storageSubsystem.setStorageMotor(MotorUtil.getMotorValue(MotorValue.SLOW_ACCEPT_SPEED, MotorFlip.STORAGE_FLIPPED));
                                    }
                                }
                                case 1: {
                                    // There is already a single ball loaded, run only the Acceptor
                                    // motor so the other ball is not pushed into the shooter area.
                                    storageSubsystem.setAcceptorMotor(MotorUtil.getMotorValue(MotorValue.ACCEPT_SPEED, MotorFlip.ACCEPTOR_FLIPPED));

                                    TimeUnit.MILLISECONDS.sleep(100);

                                    while (!storageSubsystem.getAcceptorSensorCovered() && !MotorUtil.isAnyStalled(processMotorGroup)) {
                                        storageSubsystem.setAcceptorMotor(MotorUtil.getMotorValue(MotorValue.ACCEPT_SPEED, MotorFlip.ACCEPTOR_FLIPPED));
                                    }

                                    TimeUnit.MILLISECONDS.sleep(250);
                                }
                            
                                // If there is more than one, do nothing because more cannot be accepted.
                                default: {}
                            }
                        }
                        case DENY: {
                            // This ball should be denied, spin the motor in the reject direction.
                            storageSubsystem.setAcceptorMotor(MotorUtil.getMotorValue(-MotorValue.ACCEPT_SPEED-0.1, MotorFlip.ACCEPTOR_FLIPPED));
        
                            // Add a slight delay to the end to make sure the ball is thrown out.
                            TimeUnit.MILLISECONDS.sleep(250);
                        }
                    }
                } catch (InterruptedException e) {}
                
                storageSubsystem.setAcceptorMotor(0);
                storageSubsystem.setStorageMotor(0);

                finished = true;
            }

            @Override
            public void colorTimeoutError() {
                storageSubsystem.setAcceptorMotor(0);
                storageSubsystem.setStorageMotor(0);
                finished = true;
            }
        });
    }
    
    @Override
    public void execute() {
        // Run the motors responsible for extending the intake out of the robot, since 
        // these connect closely together.
        enableGroup.execute();

        // In the Dashboard, a Boolean block changes color based on true/false so it can
        // possibly be flashing if there is a problem that requires interaction.
        SmartDashboard.putBoolean("Storage Error", false);
    }

    @Override
    public void end(boolean interrupted) {
        // If the command ends, possibly turn off the motor?
        storageSubsystem.setStorageMotor(0);
        storageSubsystem.setAcceptorMotor(0);

        // TODO: we may not want to immediately retract everything when command is done.
        disableGroup.execute();
    }

    @Override
    public boolean isFinished() {
        // We should return true when the ball has been successfully processed,
        // or has been timed out. Otherwise, the command is still running and
        // return false.
        return finished;
    }
}