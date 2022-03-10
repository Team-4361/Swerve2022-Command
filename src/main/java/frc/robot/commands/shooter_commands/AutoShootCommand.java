package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

import java.util.Map;

import static frc.robot.Constants.Shooter.SHOOTER_WHEEL_RADIUS;
import static frc.robot.Constants.Shooter.SHOOTER_WHEEL_MASS;
import static frc.robot.Constants.Shooter.SHOOTER_HEIGHT;
import static frc.robot.Constants.BALL_MASS;
import static frc.robot.Constants.BALL_RADIUS;

import static frc.robot.Robot.shooter;

public class AutoShootCommand extends SequentialCommandGroup {
    
    private final double requiredVelocity;
    private Map<String, Double> map;

    public AutoShootCommand() {
        
        map = Robot.shooterCamera.getTargetGoal();

        if(map.get("Status") == 0){
            this.cancel();
        }

        this.requiredVelocity = calculateVelocity(
                map.get("Pitch"),
                map.get("Distance"),
                SHOOTER_HEIGHT
        );

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Run the SensorShootCommand with the calculated velocity, after setting all the angles.
        new TimedShootCMD(5, requiredVelocity).schedule();
    }

    /**
     * 
     * @param pitchToTarget pitch to target in degrees
     * @param distanceToTarget distance to target
     * @param initialHeight initial height of ball launch
     * @return Return required shooter wheel angular velocity in RPM
     */
    private static double calculateVelocity(double pitchToTarget,
                                            double distanceToTarget,
                                            double initialHeight) {
        pitchToTarget = Math.toRadians(pitchToTarget);

        double requiredLinExtVelocity = (1 / Math.cos(pitchToTarget)) * Math.sqrt((((
            Math.pow(distanceToTarget, 2) + 
            (2.44 * distanceToTarget) + 1.4884) * 9.80)) 
            / (2 * (2.64 - initialHeight - (Math.tan(pitchToTarget) 
            * (distanceToTarget + 1.22)))));
        
        double requiredWhlAngVel = (Math.sqrt( (BALL_MASS*Math.pow(requiredLinExtVelocity, 2))/((1/2)*SHOOTER_WHEEL_MASS*Math.pow(SHOOTER_WHEEL_RADIUS, 2) + (2/5)*BALL_MASS*Math.pow(BALL_RADIUS, 2))))/(2*Math.PI);
    
        return requiredWhlAngVel*60;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
