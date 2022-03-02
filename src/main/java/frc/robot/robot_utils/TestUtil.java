package frc.robot.robot_utils;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.HashMap;

public class TestUtil {

    private HashMap<TestMode, CommandBase> defaultCommands;
    private TestMode currentTestMode;

    public enum TestMode {
        CHASSIS_DRIVE_TEST, CHASSIS_OFFSET_ADJUSTMENT, SHOOTER_ANGLE_TEST, INTAKE_ROTATION_TEST
    }

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

    public TestUtil() {
        defaultCommands = new HashMap<>();
    }
}
