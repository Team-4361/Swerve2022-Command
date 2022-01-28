package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private Supplier<ChassisSpeeds> chassisSpeedsFunction;
	private Supplier<Boolean> isLeftHanded;

  public ArcadeCommand(SwerveDriveSubsystem swerveDriveSubsystem, Supplier<ChassisSpeeds> chassisSpeedsFunction) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.chassisSpeedsFunction = chassisSpeedsFunction;
    addRequirements(swerveDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds = chassisSpeedsFunction.get();

    swerveDriveSubsystem.drive(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDriveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
