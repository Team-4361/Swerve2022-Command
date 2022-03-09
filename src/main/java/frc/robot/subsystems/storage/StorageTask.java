package frc.robot.subsystems.storage;

public enum StorageTask {
    /** Accept Task */
    ACCEPT,

    /** Reject Task */
    REJECT,

    /**
     * Neutral Task, returned when nothing should be done.
     * @see StorageSubsystem#getDetectedTask()
     */
    NEUTRAL
}
