package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

//This is for autoshoot
public class TimedShootCMD extends CommandBase {

    private double startTime;
    private final double targetTime;
    private final double shootSpeed;

    private ShootCMD shootCMD;

    /**
     * @param time       time in seconds to run the shooter
     * @param shootSpeed speed in RPM
     */
    public TimedShootCMD(double time, double shootSpeed) {
        this.targetTime = time;
        this.shootSpeed = shootSpeed;
        this.shootCMD = new ShootCMD(shootSpeed);
        this.startTime = -1;
    }

    @Override
    public void initialize() {
        shootCMD.schedule();
    }


    @Override
    public void end(boolean interrupted) {
        shootCMD.cancel();
    }

    @Override
    public boolean isFinished() {
        if (Robot.shooter.isDesiredSpeed(shootSpeed)) {
            if (startTime == -1) {
                startTime = System.currentTimeMillis();
            }

            if (startTime != -1) {
                return (System.currentTimeMillis() - startTime) > (targetTime * 1000);
            }
        }

        return false;
    }
}
