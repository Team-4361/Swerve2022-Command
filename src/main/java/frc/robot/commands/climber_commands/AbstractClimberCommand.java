package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberSubsystem;

import java.util.function.Supplier;

public abstract class AbstractClimberCommand extends CommandBase {
    private final ClimberSubsystem climber;
    private final Supplier<Boolean> isSwitchPressed;
    private final Runnable translate;

    public AbstractClimberCommand(ClimberSubsystem climber,
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
        climber.stopClimber();
    }

    @Override
    public boolean isFinished() {
        return !isSwitchPressed.get();
    }
}
