package frc.robot.commands.autonomous_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.chassis_commands.CenterShooterToHubCommand;
import frc.robot.commands.chassis_commands.MoveFWDCMD;
import frc.robot.commands.chassis_commands.MoveToBall;
import frc.robot.commands.chassis_commands.RotateCMD;
import frc.robot.commands.chassis_commands.RotateToBall;
import frc.robot.commands.intake_commands.adjustor.ExtendIntakeMagnet;
import frc.robot.commands.shooter_commands.AutoAdjustShooterAngle;
import frc.robot.commands.shooter_commands.TimedShootCMD;
import frc.robot.commands.storage_commands.SequentialStorageCMDs.IntakeProcessAccept;

public class CameraAuto extends CommandBase {


    private RotateCMD rotateCMD = new RotateCMD(0.4);

    private final SequentialCommandGroup processBallCMD = new SequentialCommandGroup(
        new ExtendIntakeMagnet(),
        new IntakeProcessAccept());

    private final SequentialCommandGroup autoShootGroup = new SequentialCommandGroup(
        new ParallelCommandGroup(new AutoAdjustShooterAngle(), new CenterShooterToHubCommand()),
        new TimedShootCMD(3, 4500)
    );

    private ParallelCommandGroup navigateToBall = new ParallelCommandGroup(new SequentialCommandGroup(new RotateToBall(), new MoveToBall()), processBallCMD);
    

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        if(Robot.chassisCamera.getTargetGoal().get("Status") == 0 && Robot.storage.getBallsLoaded() == 0){
            if(!rotateCMD.isScheduled()){
                rotateCMD.schedule();
            }
        } else if(Robot.chassisCamera.getTargetGoal().get("Status") == 1 && Robot.storage.getBallsLoaded() == 0){
            rotateCMD.cancel();

            if(!navigateToBall.isScheduled()){
                navigateToBall.schedule();
            }
        } else if (Robot.storage.getBallsLoaded() > 0){
            rotateCMD.cancel();
            if(!autoShootGroup.isScheduled()){
                autoShootGroup.schedule();
            }
        }
    }
}