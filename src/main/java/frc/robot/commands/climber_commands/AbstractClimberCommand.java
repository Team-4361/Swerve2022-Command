package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.LeftClimberSubsystem;

import java.util.function.Supplier;

public abstract class AbstractClimberCommand extends CommandBase {
    private final LeftClimberSubsystem climber;
    private final Supplier<Boolean> isSwitchPressed;
    private final Runnable translate;

    public AbstractClimberCommand(LeftClimberSubsystem climber,
                                  Supplier<Boolean> isSwitchPressed,
                                  Runnable translate) {
        this.climber = climber;
        this.isSwitchPressed = isSwitchPressed;
        this.translate = translate;
    }

    @Override
    public void initialize() {
        addRequirements(climber);
    }

    @Override
    public void execute() {
        translate.run();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return !isSwitchPressed.get();
    }
}
