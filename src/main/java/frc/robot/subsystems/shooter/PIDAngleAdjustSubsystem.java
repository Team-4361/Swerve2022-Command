package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot_utils.motor.MotorUtil;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.robot_utils.encoder.ConcurrentRotationalEncoder;
import me.wobblyyyy.pathfinder2.geometry.Angle;
import me.wobblyyyy.pathfinder2.revrobotics.SparkMaxMotor;

import static frc.robot.Constants.MotorFlip.ADJUSTOR_FLIPPED;
import static frc.robot.Constants.ShooterAdjustor.ADJUSTOR_GEAR_RATIO;
import static frc.robot.Constants.ShooterAdjustor.ADJUSTOR_MOTOR_ID;

import java.util.Map;

public class PIDAngleAdjustSubsystem extends SubsystemBase {
    private final SparkMaxMotor adjustor;
    private final ConcurrentRotationalEncoder absoluteEncoder;
    private final PIDController controller;
    private Angle targetAngle;
    //private final DigitalInput adjustorLimit = new DigitalInput(ADJUSTOR_LIMIT_PORT);

    public PIDAngleAdjustSubsystem() {
        adjustor = SparkMaxMotor.brushless(ADJUSTOR_MOTOR_ID, ADJUSTOR_FLIPPED);
        absoluteEncoder = new ConcurrentRotationalEncoder(adjustor.getSpark())
                .setFlipped(ADJUSTOR_FLIPPED)
                .setRPMTolerance(0.5);

        controller = new PIDController((double) 1 / 63, 0, 0);
        controller.setSetpoint(0.0);
        
        targetAngle = new Angle();
    }

    public void zero() {
        absoluteEncoder.reset();
    }

    public SparkMaxMotor getAdjustor() {
        return this.adjustor;
    }

    public double rotationToAngle(double rotation) {
        return ((Math.abs(rotation) * 360) * (1 / ADJUSTOR_GEAR_RATIO));
    }

    public double getAngle() {
        return rotationToAngle(Math.abs(
                    absoluteEncoder.getAbsoluteRotations()));
    }

    public boolean atDesiredAngle(double desired,
                                  double tolerance) {
        return (MotorUtil.inTolerance(desired, getAngle(), tolerance));
    }

    public void setAngle(double angle) {
        targetAngle = Angle.fixedDeg(angle);
    }

    public void setRotationsFromBase(double position) {
        setAngle(rotationToAngle(position));
    }

    // public boolean getAdjustorLimit(){
    //     return adjustorLimit.get();
    // }

    @Override
    public void periodic() {
        absoluteEncoder.periodic();
        Angle currentAngle = Angle.fixedDeg(getAngle());
        double delta = Angle.minimumDelta(currentAngle, targetAngle);
        double adjustorMotorPower = controller.calculate(delta);
        adjustor.setPower(adjustorMotorPower);

        SmartDashboard.putNumber("pid angle", getAngle());
        SmartDashboard.putString("pid target", targetAngle.toString());
        SmartDashboard.putNumber("pid velocity", absoluteEncoder.getVelocity());
    }
}
