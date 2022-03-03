package frc.robot.commands.storage_commands.SequentialStorageCMDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.MotorValue;

public class StorageRunAcceptor extends CommandBase{
    
    private double acceptorPower = MotorValue.ACCEPT_SPEED;
    private double time = 0.5;

    private double startTime;

    /**
     * 
     * @param value motor power
     * @param time  time in seconds
     */
    public StorageRunAcceptor(double value, double time){
        acceptorPower = value;
        this.time = time;
    }

    /**
     * 
     * @param value motor power
     */
    public StorageRunAcceptor(double value){
        acceptorPower = value;
    }

    public StorageRunAcceptor() {}
    


    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        addRequirements(Robot.storage);

        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        Robot.storage.setAcceptorMotor(acceptorPower);
    }

    private double getElapsedTime(){
        return System.currentTimeMillis() - startTime; 
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        Robot.storage.setAcceptorMotor(0);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return getElapsedTime() >= (this.time*1000);
    }
}
