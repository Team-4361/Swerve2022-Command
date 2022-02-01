package frc.robot;

import edu.wpi.first.wpilibj.I2C.Port;

public class Constants {
    public static final int XY_STICK_ID = 0;
    public static final int Z_STICK_ID = 1;
    public static final int CONTROLLER_ID = 2;

    public static final double FR_OFFSET = 0.790 - (2* Math.PI)- (Math.PI/2);
    public static final double FL_OFFSET = -2.95 - (Math.PI/2) - (Math.PI);
    public static final double BR_OFFSET = -1.7 - (Math.PI/2);
    public static final double BL_OFFSET = 0.111 - (2*Math.PI) - (Math.PI/2) - (Math.PI);

    public static final double DEADZONE = 0.05;
    public static final int LEFT_HANDED_BUTTON = 9;

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


    // Intake Subsystem
    public static final int INTAKE_PORT = 0;
    public static final int ACCEPTOR_PORT = 0;
    public static final Port COLOR_SENSOR_PORT = Port.kOnboard;
    public static final int PHOTOELECTRIC_DIO = 0;
    public static final int PHOTOELECTRIC_DIO_2 = 0;

    // -1.0 to 1.0 (full speed)
    public static final double INTAKE_MOTOR_SPEED = 1.0;
    public static final double ACCEPTOR_MOTOR_SPEED = 1.0;

    private Constants() {

    }
}
