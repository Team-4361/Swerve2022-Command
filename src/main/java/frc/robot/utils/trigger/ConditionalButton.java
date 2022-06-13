package frc.robot.utils.trigger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.function.Supplier;

public class ConditionalButton extends JoystickButton {
    private Supplier<Boolean> conditionSupplier;

    /**
     * Creates a joystick button for triggering commands.
     *
     * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick, etc.)
     * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
     */
    public ConditionalButton(GenericHID joystick, int buttonNumber) {
        super(joystick, buttonNumber);
    }

    /**
     * Creates a joystick button for triggering commands, with a default {@link Supplier} for the condition.
     *
     * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
     * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
     */
    public ConditionalButton(GenericHID joystick, int buttonNumber, Supplier<Boolean> supplier) {
        super(joystick, buttonNumber);
        setSupplier(supplier);
    }

    /**
     * Sets the {@link Supplier} that is used to calculate when the button should be "pressed".
     * @return {@link ConditionalButton} instance.
     */
    public ConditionalButton setSupplier(Supplier<Boolean> supplier) {
        this.conditionSupplier = supplier;
        return this;
    }

    /** @return If the button should be "pressed" */
    @Override
    public boolean get() {
        return this.conditionSupplier.get();
    }
}
