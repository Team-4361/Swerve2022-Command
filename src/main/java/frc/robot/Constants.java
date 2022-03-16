package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import frc.robot.robot_utils.TestUtil;
import frc.robot.subsystems.storage.RetractMode;

public class Constants {

    public static double BALL_MASS = 0.2676195;
    public static double BALL_RADIUS = 0.2413;

    public static boolean GET_DATA = false;


    public static class Control {
        public static final int XY_STICK_ID = 0;
        public static final int Z_STICK_ID = 1;
        public static final int CONTROLLER_ID = 2;

        public static final int /* Xbox Controller Map */
                // BUTTONS
                XBOX_X = 3,
                XBOX_Y = 4,
                XBOX_B = 2,
                XBOX_A = 1,
                XBOX_START = 8,
                XBOX_END = 10,

        // DPAD
        XBOX_DPAD_LEFT = 14,
                XBOX_DPAD_RIGHT = 15,
                XBOX_DPAD_UP = 12,
                XBOX_DPAD_DOWN = 13,

        // CONTROLLER
        XBOX_LEFT_STICK = 9,
                XBOX_RIGHT_STICK = 10,

        // TOP BUTTONS
        XBOX_RIGHT_TRIGGER = 6,
                XBOX_LEFT_TRIGGER = 5;
    }

    public static class MotorFlip {
        // TODO: May need to be changed later.
        public final static boolean ACCEPTOR_FLIPPED = false;
        public final static boolean STORAGE_FLIPPED = false;
        public final static boolean SHOOTER_FLIPPED = true;
        public static final boolean CLIMBER_RIGHT_FLIPPED = false;
        public static final boolean CLIMBER_LEFT_FLIPPED = false;

        public final static boolean ADJUSTOR_FLIPPED = false;

        public final static boolean INTAKE_FLIPPED = true;
        public final static boolean INTAKE_EXTENDER_LEFT_FLIPPED = true;
        public final static boolean INTAKE_EXTENDER_RIGHT_FLIPPED = true;
    }

    public static class TestValue {
        // You can change the default Testing mode to be run here, when Driver Station
        // is in testing mode.
        public final static TestUtil.TestMode DEFAULT_TEST_MODE = TestUtil.TestMode.INTAKE_ROTATION_TEST;

        // TODO: adjust enablers based on how robot is completed
        public final static boolean DRIVE_ENABLED = true;
        public final static boolean CAMERA_ENABLED = false;
    }

    public static class Chassis {
        public static final double FR_OFFSET = -2.38 - (2 * Math.PI) + (Math.PI);
        public static final double FL_OFFSET = -9.401 - (Math.PI / 2);
        public static final double BR_OFFSET = -3.345 - (Math.PI / 2) - (2 * Math.PI);
        public static final double BL_OFFSET = -6.12 - (2 * Math.PI) - (Math.PI / 2);

        public static final double CONTROLLER_DEADZONE = 0.1;

        // In meters
        public static final double SWERVE_CHASSIS_SIDE_LENGTH = 0.762;

        public static final int FR_DRIVE_ID = 4;
        public static final int FL_DRIVE_ID = 2;
        public static final int BR_DRIVE_ID = 8;
        public static final int BL_DRIVE_ID = 6;

        public static final int FR_TURN_ID = 3;
        public static final int FL_TURN_ID = 1;
        public static final int BR_TURN_ID = 7;
        public static final int BL_TURN_ID = 5;

        public static final int FR_DIO_ENCODER_PORT = 3;
        public static final int FL_DIO_ENCODER_PORT = 0;
        public static final int BR_DIO_ENCODER_PORT = 2;
        public static final int BL_DIO_ENCODER_PORT = 1;

        /**
         * the radius of each of the swerve drive wheels (meters)
         */
        public static final double SWERVE_WHEEL_RADIUS = 0.0508;
        /**
         * the circumference of each of the swerve drive wheels (meters)
         */
        public static final double SWERVE_WHEEL_CIRCUMFERENCE =
            SWERVE_WHEEL_RADIUS * 2 * Math.PI;
        public static final double ODOMETRY_MS_INTERVAL = 5;

        public static final double BL_ERROR_FACTOR = 1;
        public static final double BR_ERROR_FACTOR = 1;
        public static final double FR_ERROR_FACTOR = 1;
        public static final double FL_ERROR_FACTOR = 1;
    }

    public static class Climber {
        public static final int L_CLIMBER_ID = 21;
        public static final int R_CLIMBER_ID = 10;

        // bottom magnet
        public static final int BL_LIMIT_ID = 6;
        public static final int BR_LIMIT_ID = 7;

        public static final int TL_LIMIT_ID = 23;
        public static final int TR_LIMIT_ID = 22;
    }

    public static class Intake {
        public static final int INTAKE_SPIN_MOTOR_ID = 14;

        // DIO
        public static final int BL_MAGNET_ID = 8;
        public static final int BR_MAGNET_ID = 9;

        // TODO: change
        public static final int FL_MAGNET_ID = 19;
        public static final int FR_MAGNET_ID = 18;

        public static final int L_INTAKE_EXTEND_ID = 12;
        public static final int R_INTAKE_EXTEND_ID = 11;

        // TODO: find correct rotations that you want, subtract from total rotations
        public static final double INTAKE_EXTEND_SETPOINT = -7;
        public static final double INTAKE_RETRACT_SETPOINT = -0.5;



        // TODO: this is the amount subtracted from INTAKE_TOTAL_EXTEND_ROTATIONS and is used for
        // TODO: the amount to use from zero when retracting
        public static final double INTAKE_ROTATION_BUFFER = 3;

        public static final boolean LIMIT_SWITCH_ENABLED = true;

        /** Gives you some time to react before switching modes, disable
         * when you know it works. */
        public static final boolean TEST_SAFETY_ENABLED = true;
    }

    public static class ShooterAdjustor {
        public static final double ADJUSTOR_GEAR_RATIO = /*1:*/160;
        public static final int ADJUSTOR_MOTOR_ID = 15;

        public static final int ADJUSTOR_LIMIT_PORT = 20;

        public static final boolean LIMIT_SWITCH_ENABLED = false;

        // in degrees
        public static final double ADJUSTOR_ANGLE_MAX = 29;
        public static final double ADJUSTOR_ANGLE_MIN = 0;
    }

    public static class Shooter {
        public static final int SHOOTER_MOTOR_ID = 17;
        public static final double SHOOTER_WHEEL_RADIUS = 0.0889;
        public static final double DESIRED_RPM = 4800;
        //In KG
        public static double SHOOTER_WHEEL_MASS = 0.603844;
        public static double SHOOTER_HEIGHT = 0.5;
    }

    public static class Storage {
        /**
         * The first motor in the Storage device, used to accept the ball after the sensor is activated.
         */
        public static final int ACCEPTOR_MOTOR_PORT = 13;

        /**
         * The second middle motor in the Storage device, used to move the ball inside.
         */
        public static final int STORAGE_MOTOR_PORT = 16;

        /**
         * Used to detect the presence of a ball inside the front of the device, mainly used for 2nd entering ball.
         */
        public static final int ACCEPTOR_PHOTO_ELECTRIC_PORT = 4;

        /**
         * Used to detect the presence of a ball inside the middle of the device, where the first ball should go.
         */
        public static final int STORAGE_PHOTO_ELECTRIC_PORT = 5;

        /**
         * The port to use for the Color Sensor detection.
         */
        public static final I2C.Port COLOR_SENSOR_PORT = I2C.Port.kOnboard;

        // Adjust based on sensitivity.
        public static final double BLUE_THRESHOLD = 0.325;//0.30
        public static final double RED_THRESHOLD = 0.28;//0.265
        public static final double PROXIMITY_THRESHOLD = 120;

        public static final int STORAGE_EXTRA_TIME_MS = 500;

        public static final double LENGTH_ROD_TO_ANGULAR_POS = 0;

        public static final RetractMode RETRACT_MODE_FINISHED = RetractMode.RETRACT_ALWAYS;
        public static final boolean RETRACT_ON_ACCEPT = true;
    }

    //PhotonVision Constants
    public static class ShooterCameraConsts {
        public static final String CAMERA_NAME = "RoxShooterCam";
        public static final double CAMERA_HEIGHT = 0.8001;
        public static final double CAMERA_PITCH = Math.PI/4;
        public static final double TAPE_HEIGHT = 2.54;
    }

    public static class ChassisCameraConsts {
        public static final String CAMERA_NAME = "RoxBallCam";
        public static final double CAMERA_HEIGHT = 0.5715;
        public static final double CAMERA_PITCH = 0.0;
        public static final double BALL_HEIGHT = 0.2413;
    }

    // These values are designed to be changed based on the Motor
    public static class MotorValue {
        public static final double SHOOT_SPEED = 1.0;
        public static final double ACCEPT_SPEED = 0.4;
        public static final double SPIN_INTAKE_ACCEPT = 0.6;
        public static final double SLOW_ACCEPT_SPEED = 0.25;

        public static final double EXTERNAL_ACCEPT_SPEED = 1;

        public static final double ADJUSTOR_SPEED = 0.3;

        public static final double CLIMBER_SPEED = 1.0;

        // Stall current in amps, stops the motor when the current rises above
        // the maximum value.
        public final static double STALL_CURRENT = 80; /* AMPS */

        // Stall RPM, stops the motor when the RPM drops below + current above limit.
        public final static double STALL_RPM = 2000; /* RPM */

        // Used for stall protection, disable if any issues occur from it.
        public final static boolean CURRENT_MEASURING = false;
    }
}
