package frc.robot.robot_utils;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.HashMap;

public class TestUtil {

    // Add testing modes
    public enum TestMode { 
        CHASSIS_DRIVE_TEST, 
        CHASSIS_OFFSET_ADJUSTMENT,
        SHOOTER_ANGLE_TEST;
    }

    private HashMap<TestMode, CommandBase> defaultCommands = new HashMap<>();
    private TestMode currentTestMode;

    public TestUtil setTestMode(TestMode mode) {
        currentTestMode = mode;
        return this;
    }

    public TestMode getTestMode() {
        return currentTestMode;
    }

    public CommandBase getExecutedCommand() {
        return defaultCommands.get(currentTestMode);
    }

    public TestUtil addDefaultCommand(TestMode mode, CommandBase command) {
        defaultCommands.put(mode, command);
        return this;
    }

    public void runExecutedCommand() {
        defaultCommands.get(currentTestMode).schedule();
    }

    public TestUtil setDefaultCommands(HashMap<TestMode, CommandBase> commands) {
        defaultCommands = commands;
        return this;
    }

    public TestUtil(HashMap<TestMode, CommandBase> commands) {
        defaultCommands = commands;
    }

    public TestUtil() {}
}
