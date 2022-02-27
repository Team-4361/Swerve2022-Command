package frc.robot.subsystems.storage;

public interface TaskListener {
    void onAcceptTask();
    void onRejectTask();
    void onTimeout();
}
