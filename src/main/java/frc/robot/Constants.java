package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import frc.robot.robot_utils.TestUtil;

// TODO: check all values and make sure they are correct
public class Constants {

    public static class Control {
        public static final int XY_STICK_ID = 0;
        public static final int Z_STICK_ID = 1;
        public static final int CONTROLLER_ID = 2;
    }

    public static class MotorFlip {
        // TODO: May need to be changed later.
        public final static boolean ACCEPTOR_FLIPPED = true;
        public final static boolean STORAGE_FLIPPED = false;
        public final static boolean SHOOTER_FLIPPED = false;
        public final static boolean CLIMBER_FLIPPED = false;
        public final static boolean ADJUSTOR_FLIPPED = false;
    }

    public static class TestValue {
        // You can change the default Testing mode to be run here, when Driver Station
        // is in testing mode.
        public final static TestUtil.TestMode DEFAULT_TEST_MODE = TestUtil.TestMode.SHOOTER_ANGLE_TEST;
    }

    public static class Chassis {
        public static final double FR_OFFSET = -4.94 - (2* Math.PI)- (Math.PI/2);
        public static final double FL_OFFSET = -6.23 - (Math.PI/2);
        public static final double BR_OFFSET = -1.03 - (Math.PI/2) - (2*Math.PI);
        public static final double BL_OFFSET = -3.02  - (2*Math.PI) - (Math.PI/2);
        

        public static final double DEAD_ZONE = 0.05;

        //In meters
        public static final double SWERVE_CHASSIS_SIDE_LENGTH = 0.762;

        public static final int FR_DRIVE_ID = 4;
        public static final int FL_DRIVE_ID = 2;
        public static final int BR_DRIVE_ID = 8;
        public static final int BL_DRIVE_ID = 6;

        public static final int FR_TURN_ID = 3;
        public static final int FL_TURN_ID = 1;
        public static final int BR_TURN_ID = 7;
        public static final int BL_TURN_ID = 5;

        public static final int FR_DIO_ENCODER_PORT = 1;
        public static final int FL_DIO_ENCODER_PORT = 0;
        public static final int BR_DIO_ENCODER_PORT = 3;
        public static final int BL_DIO_ENCODER_PORT = 2;

        public static final double SWERVE_WHEEL_RADIUS = 0.0508;
    }

    public static class Climber {
        public static final int L_CLIMBER_PORT = 8;
        public static final int R_CLIMBER_PORT = 5;

        public static final int B_CLIMBER_SWITCH = 0;
        public static final int T_CLIMBER_SWITCH = 0;
    }

    public static class IntakeShooter {
        /**
         * The first motor in the Storage device, used to accept the ball after the sensor is activated.
         */
        public static final int ACCEPTOR_MOTOR_PORT = 7;

        // The second middle motor in the Storage device, used to move the ball inside.
        public static final int STORAGE_MOTOR_PORT = 6;

        // The shooter motor in the Shooter device, runs at Full Speed and shoots the ball.
        public static final int SHOOTER_MOTOR_PORT = 8;

        // Left Intake Extend Motor Port
        public static final int L_INTAKE_MOTOR_ID = 9; 

        // Right Intake Extend Motor Port
        public static final int R_INTAKE_MOTOR_ID = 11;

        public static final int MAX_INTAKE_MOTOR_POSITION = 0;

        public static final int INTAKE_MOTOR_ID = 12;

        public static final int SHOOTER_WHEEL_RADIUS = 0;

        ////////////////////////////////////////////////////////////////////////////////////////
            
        // Used to detect the presence of a ball inside the front of the device, mainly used for 2nd entering ball.
        public static final int ACCEPTOR_PHOTO_ELECTRIC_PORT = 1;
    
        // Used to detect the presence of a ball inside the middle of the device, where the first ball should go.
        public static final int STORAGE_PHOTO_ELECTRIC_PORT = 0;
    
        ////////////////////////////////////////////////////////////////////////////////////////
    
        // The port to use for the Color Sensor detection.
        public static final I2C.Port COLOR_SENSOR_PORT = I2C.Port.kOnboard;

        public static final double ADJUSTOR_GEAR_RATIO = /*1:*/160;

    
        // Adjust based on sensitivity.
        public static final double BLUE_THRESHOLD = 0.30;
        public static final double RED_THRESHOLD = 0.30;
        public static final double PROXIMITY_THRESHOLD = 120;

        public static final double LENGTH_ROD_TO_ANGULAR_POS = 0;

        // Shooter Adjustment Motor
        public static final int SHOOTER_ADJUSTMENT_PORT = 0;
    }
    
    //PhotonVision Constants
    public static class ShooterCameraConsts {
        public static final String NETWORK_TABLE_HOSTNAME = "";
        public static final String CAMERA_NAME = "";
	    public static final double CAMERA_HEIGHT = 0.0;
        public static final double CAMERA_PITCH = 0.0;
        public static final double TAPE_HEIGHT = 0.0;
    }

    public static class ChassisCameraConsts {
        public static final String CAMERA_NAME = "";
	    public static final double CAMERA_HEIGHT = 0.0;
        public static final double CAMERA_PITCH = 0.0;
        public static final double BALL_HEIGHT = 0.0;
    }
    
    // These values are designed to be changed based on the Motor 
    public static class MotorValue {
        public static final double SHOOT_SPEED = 1.0;
        public static final double ACCEPT_SPEED = 0.3;
        public static final double SLOW_ACCEPT_SPEED = 0.2;

        public static final double ADJUSTOR_SPEED = 0.3;

        public static final double CLIMBER_SPEED = 0.5;

        // Stall current in amps, stops the motor when the current rises above
        // the maximum value.
        public final static double STALL_CURRENT = 80;
        
        // Stall RPM, stops the motor when the RPM drops below + current above limit.
        public final static double STALL_RPM = 2000;

        // Target RPM for the Shooter Motor to activate loading
        public final static double SHOOTER_TARGET_RPM = 4800;

        // Used for stall protection, disable if any issues occur from it.
        public final static boolean CURRENT_MEASURING = true;
    }
}
