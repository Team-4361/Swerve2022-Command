package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.motor.MotorUtil;

import java.util.Map;

import static java.util.Map.entry;


public class Constants {
    /**
     * This {@link Power} class is designed to hold the Breaker Numbers for either a Power Distribution Hub (PDH), or
     * a Power Distribution Panel (PDP). Assuming these numbers are correct, it will be able to provide detailed
     * power logging information by each circuit, and their max currents.
     *
     */
    public static class Power {
        public static int FL_ROTATION_FUSE = 15;
        public static int FL_DRIVE_FUSE = 16;
        public static int FR_ROTATION_FUSE = 9;
        public static int FR_DRIVE_FUSE = 8;

        public static int BL_ROTATION_FUSE = 19;
        public static int BL_DRIVE_FUSE = 18;

        public static int BR_ROTATION_FUSE = 6;
        public static int BR_DRIVE_FUSE = 5;

        public static int LEFT_CLIMBER_FUSE = 17;
        public static int RIGHT_CLIMBER_FUSE = 7;

        public static int LEFT_INTAKE_EXTENDER_FUSE = 13;
        public static int RIGHT_INTAKE_EXTENDER_FUSE = 14;

        public static int STORAGE_ACCEPT_FUSE = 1;
        public static int EXTERNAL_INTAKE_FUSE = 4;

        public static int SHOOTER_HOOD_FUSE = 2;
        public static int STORAGE_FEEDER_FUSE = 3;

        public static int SHOOTER_FLYWHEEL_FUSE = 0;

        /**
         * These are the entries that are made to store each <b>used breaker number</b>, and the max
         * ampere rating for it.
         *
         * <p>
         *     In each {@link java.util.Map#entry(Object, Object)}, both parameters should be {@link Integer} and
         *     the first number should be the Breaker ID, and the second number should be the maximum current rating.
         *
         *     <pre>
         *      Example: entry(0, 40) for Breaker #0 and a Maximum Current of 40 amps.
         *     </pre>
         * </p>
         */
        public static Map<Integer, Integer> DEFAULT_BREAKER_ENTRIES = Map.ofEntries(
                entry(0, 40),
                entry(1, 40),
                entry(3, 40),
                entry(4, 40),
                entry(5, 40),
                entry(6, 40),
                entry(7, 40),
                entry(8, 40),
                entry(9, 40),
                entry(13, 40),
                entry(14, 40),
                entry(15, 40),
                entry(16, 40),
                entry(17, 40),
                entry(18, 40),
                entry(19, 40),
                entry(20, 15),
                entry(21, 10),
                entry(22, 10)
        );
    }

    /**
     * This {@link Control} class is designed to store the variables related to the IDs of the {@link XboxController},
     * {@link Joystick}, and the button mappings designed for each..
     */
    public static class Control {
        /** The {@link Joystick} used for controlling the X and Y axis driving of the robot, typically the left Joystick. */
        public static final int XY_STICK_ID = 0;

        /** The {@link Joystick} used for controlling the Z axis driving of the robot, typically the right joystick. */
        public static final int Z_STICK_ID = 1;

        /** The {@link XboxController} ID used for controlling the <b>accessories</b> of the robot, i.e. not driving */
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

    /**
     * This {@link MotorFlip} class is designed to store information regarding if the major motors on the robot
     * should be inverted. Since this can change frequently, it is <b>important that every set of motors</b> are
     * included.
     * <br>
     * In code, you can parse these values easily by using {@link MotorUtil#flip(double, boolean)}.
     */
    public static class MotorFlip {
        // TODO: May need to be changed later.
        public final static boolean ACCEPTOR_FLIPPED = false;
        public final static boolean STORAGE_FLIPPED = false;
        public final static boolean SHOOTER_FLIPPED = true;
        public static final boolean CLIMBER_FLIPPED = true;

        public final static boolean ADJUSTOR_FLIPPED = true;

        public final static boolean INTAKE_FLIPPED = true;
        public final static boolean INTAKE_EXTENDER_FLIPPED = true;

        public final static boolean GYRO_FLIPPED = false;
    }

    /**
     * This {@link Chassis} class is designed to store values regarding Driving the robot. This includes any offsets
     * for Absolute Driving, dead-zones, and ports regarding the motors. Note that these motors usually <b>do not</b>
     * need to be flipped due to the Field Oriented driving system.
     */
    public static class Chassis {
        /** The offset of the Front Right Motor */
        public static final double FR_OFFSET = -2.38 - (2 * Math.PI) + (Math.PI);

        /** The offset of the Front Left Motor */
        public static final double FL_OFFSET = -9.401 - (Math.PI / 2);

        /** The offset of the Back Right Motor */
        public static final double BR_OFFSET = -3.345 - (Math.PI / 2) - (2 * Math.PI);

        /** The offset of the Back Left Motor */
        public static final double BL_OFFSET = -6.12 - (2 * Math.PI) - (Math.PI / 2);

        /** The dead-zone where anything below this value, nothing will happen. */
        public static final double DRIVE_DEAD_ZONE = 0.15;

        /** The length of the side of the {@link Chassis} in <b>meters.</b> */
        public static final double SWERVE_CHASSIS_SIDE_LENGTH = 0.762;

        /** The Motor ID used for the Front Right Drive Motor. */
        public static final int FR_DRIVE_ID = 4;

        /** The Motor ID used for the Front Left Drive Motor. */
        public static final int FL_DRIVE_ID = 2;

        /** The Motor ID used for the Back Right Drive Motor. */
        public static final int BR_DRIVE_ID = 8;

        /** The Motor ID used for the Back Left Drive Motor. */
        public static final int BL_DRIVE_ID = 6;

        /** The Motor ID used for the Front Right Steering Motor. */
        public static final int FR_TURN_ID = 3;

        /** The Motor ID used for the Front Left Steering Motor. */
        public static final int FL_TURN_ID = 1;

        /** The Motor ID used for the Back Right Steering Motor. */
        public static final int BR_TURN_ID = 7;

        /** The Motor ID used for the Back Left Steering Motor. */
        public static final int BL_TURN_ID = 5;

        /** The ID used for the Front Right Absolute Encoder. */
        public static final int FR_DIO_ENCODER_PORT = 3;

        /** The ID used for the Front Left Absolute Encoder. */
        public static final int FL_DIO_ENCODER_PORT = 0;

        /** The ID used for the Back Right Absolute Encoder. */
        public static final int BR_DIO_ENCODER_PORT = 2;

        /** The ID used for the Back Left Absolute Encoder. */
        public static final int BL_DIO_ENCODER_PORT = 1;

        /** The Radius of each of the Swerve Drive Wheels in <b>meters.</b> */
        public static final double SWERVE_WHEEL_RADIUS = 0.0508;

        /** The Circumference of each of the Swerve Drive Wheels in <b>meters.</b> */
        public static final double SWERVE_WHEEL_CIRCUMFERENCE = SWERVE_WHEEL_RADIUS * 2 * Math.PI;

        /** How often the Odometry tracking of the Swerve Drive System is updated in <b>milliseconds.</b> */
        public static final double ODOMETRY_MS_INTERVAL = 5;

        /** The Error Factor for the Back Left. */
        public static final double BL_ERROR_FACTOR = 1;

        /** The Error Factor for the Back Right. */
        public static final double BR_ERROR_FACTOR = 1;

        /** The Error Factor for the Front Right. */
        public static final double FR_ERROR_FACTOR = 1;

        /** The Error Factor for the Front Left. */
        public static final double FL_ERROR_FACTOR = 1;
    }

    /**
     * This {@link Climber} class is designed to hold the IDs responsible for the {@link Climber}. This includes
     * the Motor IDs, and the Safety Limit Switch {@link DigitalInput}.
     */
    public static class Climber {
        /** The motor ID used for the Left Climbing Rod. */
        public static final int L_CLIMBER_ID = 21;

        /** The motor ID used for the Right Climbing Rod. */
        public static final int R_CLIMBER_ID = 10;

        /** The {@link DigitalInput} used for the Back Left Safety. */
        public static final int BL_LIMIT_ID = 6;

        /** The {@link DigitalInput} used for the Back Right Safety. */
        public static final int BR_LIMIT_ID = 7;

        /** The {@link DigitalInput} used for the Top Left Safety. */
        public static final int TL_LIMIT_ID = 23;

        /** The {@link DigitalInput} used for the Top Right Safety. */
        public static final int TR_LIMIT_ID = 22;
    }

    /**
     * This {@link Intake} class is designed to hold the values regarding the front Intake on the Robot, including
     * the motor IDs, magnet stop IDs, and other values.
     */
    public static class Intake {
        /** The Motor ID used for spinning the outside Intake Wheels. <b>Not for extending the Intake.</b> */
        public static final int INTAKE_SPIN_MOTOR_ID = 14;

        /** The {@link DigitalInput} used for the Back Left Stopper. */
        public static final int BL_MAGNET_ID = 8;

        /** The {@link DigitalInput} used for the Back Right Stopper. */
        public static final int BR_MAGNET_ID = 9;

        /** The {@link DigitalInput} used for the Front Left Stopper. */
        public static final int FL_MAGNET_ID = 19;

        /** The {@link DigitalInput} used for the Front Right Stopper. */
        public static final int FR_MAGNET_ID = 18;

        /** The Motor ID used for the Left Intake Extender. */
        public static final int L_INTAKE_EXTEND_ID = 12;

        /** The Motor ID used for the Right Intake Extender. */
        public static final int R_INTAKE_EXTEND_ID = 11;

        /** The amount of rotations to extend the Intake when the Magnets are disabled (or broken). */
        public static final double INTAKE_EXTEND_SETPOINT = -7;

        /** The amount of rotations to retract the Intake when the Magnets are disabled (or broken). */
        public static final double INTAKE_RETRACT_SETPOINT = -0.5;

        /** This is the amount subtracted from {@link #INTAKE_EXTEND_SETPOINT} and is used for
        the amount to use from zero when retracting */
        public static final double INTAKE_ROTATION_BUFFER = 3;

        /** If the Magnets are enabled and will be used during operation. Disable if they cause any problems. /*/
        public static final boolean MAGNET_ENABLED = true;

        /** Gives you some time to react before switching modes, disable
         * when you know it works. */
        public static final boolean TEST_SAFETY_ENABLED = true;
    }

    /**
     * This {@link ShooterAdjustor} class is designed to hold values regarding adjusting the Shooter to match
     * a specific angle. This includes the adjustment gear ratio, motor IDs, top limit switch, and speed.
     */
    public static class ShooterAdjustor {
        /**
         * The gear ratio of the Adjustor Motor (1) and the gear used to move the adjustor mechanism up and down.
         */
        public static final double ADJUSTOR_GEAR_RATIO = /*1:*/160;

        /** The Motor ID used for the Adjustor Motor. */
        public static final int ADJUSTOR_MOTOR_ID = 15;

        /** The limit switch used for telling the Adjustor when the maximum angle is reached. */
        public static final int ADJUSTOR_LIMIT_PORT = 20;

        /** The maximum speed to move the Adjustor, this should not be too high. */
        public static final double ADJUSTOR_SPEED = 0.06;

        /** How many degrees per each rotation of the motor will be changed. */
        public static final double DEGREES_PER_ROTATION = 2.25d;

        /** The maximum amount of times the motor is able to rotate before maxing out. */
        public static double MAX_ROTATION = 14;

        /** The maximum angle of the Adjustor. */
        public static final double ADJUSTOR_ANGLE_MAX = 30;
    }

    /**
     * This {@link Shooter} class is designed to hold values related to the shooter, such as the default desired
     * RPM, ports, and others.
     */
    public static class Shooter {
        public static final int SHOOTER_MOTOR_ID = 17;
        public static final double SHOOTER_WHEEL_RADIUS = 0.0889;
        public static final double DESIRED_RPM = 4800;
        public static final double FEED_FWD = 1.7e-4 * (12.5/12);

        //In KG
        public static double SHOOTER_WHEEL_MASS = 0.603844;
        public static double SHOOTER_HEIGHT = 0.5;
    }

    public static class Storage {
        /** The first motor in the Storage device, used to accept the ball after the sensor is activated. */
        public static final int ACCEPTOR_MOTOR_PORT = 13;

        /** The second middle motor in the Storage device, used to move the ball inside. */
        public static final int STORAGE_MOTOR_PORT = 16;

        /** Used to detect the presence of a ball inside the front of the device, mainly used for 2nd entering ball. */
        public static final int ACCEPTOR_PHOTO_ELECTRIC_PORT = 4;

        /** Used to detect the presence of a ball inside the middle of the device, where the first ball should go. */
        public static final int STORAGE_PHOTO_ELECTRIC_PORT = 5;

        /** The port to use for the Color Sensor detection. */
        public static final I2C.Port COLOR_SENSOR_PORT = I2C.Port.kOnboard;

        /** The minimum reading for the Blue Color Sensor that will activate the pulling sequence */
        public static final double BLUE_THRESHOLD = 0.31;

        /** The minimum reading for the Red Color Sensor that will activate the pulling sequence */
        public static final double RED_THRESHOLD = 0.25;

        /** The minimum reading for the Proximity Sensor + the required Color Sensor to start retracting. */
        public static final double PROXIMITY_THRESHOLD = 120;

        /**
         * How many milliseconds to "hold" the Storage System after all operations are complete before fully
         * stopping. This can remove some imperfections when retracting the ball.
         */
        public static final int STORAGE_EXTRA_TIME_MS = 500;

        public static final double LENGTH_ROD_TO_ANGULAR_POS = 0;

        /** If the Storage mechanism should be retracted automatically on a successful operation. */
        public static final boolean RETRACT_ON_ACCEPT = true;
    }

    /**
     * This {@link MotorValue} class is designed to hold values relating to the <b>default motor speeds.</b>
     */
    public static class MotorValue {
        public static final double SHOOT_SPEED = 1.0;
        public static final double ACCEPT_SPEED = 0.25;
        public static final double SPIN_INTAKE_ACCEPT = 0.6;
        public static final double SLOW_ACCEPT_SPEED = 0.25;
        public static final double EXTERNAL_ACCEPT_SPEED = 1;
        public static final double ADJUSTOR_SPEED = 0.3;
        public static final double CLIMBER_SPEED = 1.0;
    }
}
