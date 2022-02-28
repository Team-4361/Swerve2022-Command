package frc.robot.subsystems.storage;

import frc.robot.commands.storage_commands.SequentialStorageCMDs.StorageAcceptBall;
import frc.robot.commands.storage_commands.SequentialStorageCMDs.StorageRejectBall;

/**
 * This is designed to be used as a configurable Intake retraction mode,
 * at the end of a Storage cycle. Either always, when full, or never.
 *
 * @see StorageAcceptBall#end(boolean)
 * @see StorageRejectBall#end(boolean)
 */
public enum RetractMode {
    /** Used when you want to always retract the intake after receiving a ball. */
    RETRACT_ALWAYS,

    /** Used when you want to only retract the intake when no more balls can be physically received. */
    RETRACT_WHEN_FULL,

    /** Used when you never want to retract the intake. */
    RETRACT_NEVER
}
