package frc.robot.robot_utils.trigger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;

public class TriggerButtonLeft extends JoystickButton{

    
    public TriggerButtonLeft(GenericHID joystick, int buttonNumber) {
        super(joystick, buttonNumber);
        //TODO Auto-generated constructor stub
    }

    @Override
    public boolean get() {
        // TODO Auto-generated method stub
        return RobotContainer.controller.getLeftTriggerAxis() > 0.8;
    }
    
}
