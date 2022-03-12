package frc.robot.robot_utils.trigger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.ArrayList;

/**
 * This class is designed to be able to have multiple triggers bound to it,
 * and it will be easier to use for {@link edu.wpi.first.wpilibj2.command.button.JoystickButton#whenHeld(Command)}
 */
public class MultiTrigger extends Trigger {
    private Trigger[] triggers;

    public MultiTrigger(Trigger... triggers) {
        this.triggers = triggers;
    }

    @Override
    public boolean get() {
        boolean triggerPressed = true;
        for (Trigger trigger: this.triggers) {
            if (!trigger.get()) {
                triggerPressed = false;
            }
        }
        return triggerPressed;
    }
}
