package frc.robot.commands.chassis_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import java.util.HashMap;

<<<<<<< HEAD:src/main/java/frc/robot/commands/chassis_commands/MoveLeftCMD.java
public class MoveLeftCMD extends CommandBase {

=======
public class TestChassisCMD extends CommandBase {

    private final boolean moveRight = true;
    private boolean moveLeft, moveFoward, moveBackward;
>>>>>>> 3587b6a18f88668141704d9ae3980ae035997951:src/main/java/frc/robot/commands/chassis_commands/TestChassisCMD.java

    private final PIDController chassisController;

    private final double distanceToMove = 2.4;
    private double initDriveEncoderDist = 0;

<<<<<<< HEAD:src/main/java/frc/robot/commands/chassis_commands/MoveLeftCMD.java

    public MoveLeftCMD(){
=======
    public TestChassisCMD() {
>>>>>>> 3587b6a18f88668141704d9ae3980ae035997951:src/main/java/frc/robot/commands/chassis_commands/TestChassisCMD.java
        chassisController = new PIDController(0.1, 0, 0);
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void initialize() {
<<<<<<< HEAD:src/main/java/frc/robot/commands/chassis_commands/MoveLeftCMD.java
        initDriveEncoderDist = Robot.swerveDrive.getDistance();
=======

>>>>>>> 3587b6a18f88668141704d9ae3980ae035997951:src/main/java/frc/robot/commands/chassis_commands/TestChassisCMD.java
    }


    @Override
    public void execute() {
<<<<<<< HEAD:src/main/java/frc/robot/commands/chassis_commands/MoveLeftCMD.java
        double power = chassisController.calculate(getAbsDistance(), distanceToMove+0.1);

        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-power, 0, 0, Rotation2d.fromDegrees(0)));
=======

>>>>>>> 3587b6a18f88668141704d9ae3980ae035997951:src/main/java/frc/robot/commands/chassis_commands/TestChassisCMD.java
    }

    @Override
    public void end(boolean interrupted) {
<<<<<<< HEAD:src/main/java/frc/robot/commands/chassis_commands/MoveLeftCMD.java
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)));
=======

>>>>>>> 3587b6a18f88668141704d9ae3980ae035997951:src/main/java/frc/robot/commands/chassis_commands/TestChassisCMD.java
    }

    @Override
    public boolean isFinished() {
        return (getAbsDistance() > distanceToMove) ? true : false;
    }

<<<<<<< HEAD:src/main/java/frc/robot/commands/chassis_commands/MoveLeftCMD.java
    private double getAbsDistance(){
        return Robot.swerveDrive.getDistance() - initDriveEncoderDist;
=======
    private void driveRight() {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.8, 0, 0, Rotation2d.fromDegrees(0)));
    }

    private void driveLeft() {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-0.8, 0, 0, Rotation2d.fromDegrees(0)));
    }

    private void driveFWD() {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0.8, 0, Rotation2d.fromDegrees(0)));
    }

    private void driveLBack() {
        Robot.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, -0.8, 0, Rotation2d.fromDegrees(0)));
>>>>>>> 3587b6a18f88668141704d9ae3980ae035997951:src/main/java/frc/robot/commands/chassis_commands/TestChassisCMD.java
    }
}