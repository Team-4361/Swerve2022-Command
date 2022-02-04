package frc.robot.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private static final CANSparkMaxLowLevel.MotorType MOTOR_TYPE =
            CANSparkMaxLowLevel.MotorType.kBrushless;

    private static final int COUNTS_PER_REV = 4_096;

    private static final double DRIVE_PID_PROPORTIONAL = 1.0;
    private static final double DRIVE_PID_INTEGRAL = 0;
    private static final double DRIVE_PID_DERIVATIVE = 0;
    private static final double DRIVE_PID_PERIOD = 0.02;

    private static final double TURN_PID_PROPORTIONAL = 0.2;
    private static final double TURN_PID_INTEGRAL = 0.00;
    private static final double TURN_PID_DERIVATIVE = 0;
    private static final double TURN_PID_PERIOD = 0.02;

    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final DutyCycleEncoder rotationPWMEncoder;
    private final double offset;

    private final PIDController driveController = new PIDController(
            DRIVE_PID_PROPORTIONAL,
            DRIVE_PID_INTEGRAL,
            DRIVE_PID_DERIVATIVE,
            DRIVE_PID_PERIOD
    );

    private final CANSparkMax turnMotor;
    private final PIDController turnController = new PIDController(
            TURN_PID_PROPORTIONAL,
            TURN_PID_INTEGRAL,
            TURN_PID_DERIVATIVE,
            TURN_PID_PERIOD
    );

    public SwerveModule(int driveMotorId,
                        int turnMotorId, 
                        int digitalEncoderPort,
                        double offset) {
        driveMotor = new CANSparkMax(driveMotorId, MOTOR_TYPE);
        driveEncoder = driveMotor.getAlternateEncoder(COUNTS_PER_REV);

        turnMotor = new CANSparkMax(turnMotorId, MOTOR_TYPE);

        this.offset = offset;

        rotationPWMEncoder = new DutyCycleEncoder(digitalEncoderPort);
    }


    public double velocityMetersPerSecond() {
        return driveEncoder.getVelocity();
    }

    public Rotation2d getTurnAngle() {
        return new Rotation2d(turnAngleRadians());
    }

    public double turnAngleRadians() {
        return offset + (rotationPWMEncoder.get() * 2 * Math.PI);
    }

    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getTurnAngle());

        double turnPower = turnController.calculate(
                turnAngleRadians(),
                state.angle.getRadians()
        );
        
        turnMotor.set(turnPower);

        driveMotor.set(state.speedMetersPerSecond);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                velocityMetersPerSecond(),
                getTurnAngle()
        );
    }

    public void updateDashboard(String prefix) {
        String driveVelocity = String.format("%s: vel", prefix);
        String drivePower = String.format("%s: pow", prefix);
        String turnPower = String.format("%s: turn pow", prefix);
        String turnPosition = String.format("%s: turn pos", prefix);

        SmartDashboard.putNumber(driveVelocity, velocityMetersPerSecond());
        SmartDashboard.putNumber(turnPower, turnMotor.get());
        SmartDashboard.putNumber(turnPosition, getTurnAngle().getRadians());
        SmartDashboard.putNumber(drivePower, driveMotor.get());
    }
}
