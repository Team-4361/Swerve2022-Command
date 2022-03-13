package frc.robot.robot_utils.trigger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DPADUPButton extends JoystickButton{



    public DPADUPButton(XboxController controller, int buttonNumber) {
        super(controller, buttonNumber);
        //TODO Auto-generated constructor stub
    }

    @Override
    public boolean get() {
        // TODO Auto-generated method stub
        return RobotContainer.controller.getPOV() >= 315 || RobotContainer.controller.getPOV() >= 90;
    }
    
}
