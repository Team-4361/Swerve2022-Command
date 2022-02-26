package frc.robot.commands.storage_commands;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.intake_commands.RetractIntake;
import frc.robot.commands.intake_commands.SpinIntakeAccept;
import frc.robot.robot_utils.MotorUtil;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.StorageSubsystem.Task;

import java.util.concurrent.TimeUnit;

import static frc.robot.Constants.MotorFlip;
import static frc.robot.Constants.MotorValue;

public class ProcessBallCommand extends CommandBase {

    private final ParallelCommandGroup disableGroup;

    private boolean finished = false;
    private final CANSparkMax[] processMotorGroup;

    public ProcessBallCommand() {
        disableGroup = new ParallelCommandGroup()
                .alongWith(new SpinIntakeAccept())
                .alongWith(new RetractIntake());

        processMotorGroup = new CANSparkMax[]{Robot.storage.getStorageMotor(), Robot.storage.getAcceptorMotor()};

        addRequirements(Robot.storage);
    }

    @Override
    public void initialize() {
        // The intake motor should always be running while the robot is active.
        Robot.storage.addListener(new StorageSubsystem.StorageListener() {
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
                                    Robot.storage.setAcceptorMotor(MotorUtil.getMotorValue(MotorValue.ACCEPT_SPEED, MotorFlip.ACCEPTOR_FLIPPED));
                                    Robot.storage.setStorageMotor(MotorUtil.getMotorValue(MotorValue.SLOW_ACCEPT_SPEED, MotorFlip.STORAGE_FLIPPED));

                                    TimeUnit.MILLISECONDS.sleep(100);

                                    while (!Robot.storage.getStorageSensorCovered() && !MotorUtil.isAnyStalled(processMotorGroup)) {
                                        Robot.storage.setAcceptorMotor(MotorUtil.getMotorValue(MotorValue.ACCEPT_SPEED, MotorFlip.ACCEPTOR_FLIPPED));
                                        Robot.storage.setStorageMotor(MotorUtil.getMotorValue(MotorValue.SLOW_ACCEPT_SPEED, MotorFlip.STORAGE_FLIPPED));
                                    }
                                }
                                case 1: {
                                    // There is already a single ball loaded, run only the Acceptor
                                    // motor so the other ball is not pushed into the shooter area.
                                    Robot.storage.setAcceptorMotor(MotorUtil.getMotorValue(MotorValue.ACCEPT_SPEED, MotorFlip.ACCEPTOR_FLIPPED));

                                    TimeUnit.MILLISECONDS.sleep(100);

                                    while (!Robot.storage.getAcceptorSensorCovered() && !MotorUtil.isAnyStalled(processMotorGroup)) {
                                        Robot.storage.setAcceptorMotor(MotorUtil.getMotorValue(MotorValue.ACCEPT_SPEED, MotorFlip.ACCEPTOR_FLIPPED));
                                    }

                                    TimeUnit.MILLISECONDS.sleep(250);
                                }

                                // If there is more than one, do nothing because more cannot be accepted.
                                default: {
                                }
                            }
                        }
                        case DENY: {
                            // This ball should be denied, spin the motor in the reject direction.
                            Robot.storage.setAcceptorMotor(MotorUtil.getMotorValue(-MotorValue.ACCEPT_SPEED, MotorFlip.ACCEPTOR_FLIPPED));

                            // Add a slight delay to the end to make sure the ball is thrown out.
                            TimeUnit.MILLISECONDS.sleep(250);
                        }
                    }
                } catch (InterruptedException ignored) {
                }

                MotorUtil.stopMotors(processMotorGroup);

                finished = true;
                end(false);
            }

            @Override
            public void colorTimeoutError() {
                MotorUtil.stopMotors(processMotorGroup);
                finished = true;
                end(false);
            }
        });

        // In the Dashboard, a Boolean block changes color based on true/false so it can
        // possibly be flashing if there is a problem that requires interaction.
        SmartDashboard.putBoolean("Storage Error", false);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        // If the command ends, possibly turn off the motor?
        MotorUtil.stopMotors(processMotorGroup);

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