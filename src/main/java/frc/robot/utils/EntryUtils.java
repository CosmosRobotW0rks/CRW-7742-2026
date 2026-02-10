package frc.robot.utils;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;

public class EntryUtils {
    public static DoubleEntry createDoubleEntry(String entryName, double defaultValue) {
        var entry = edu.wpi.first.networktables.NetworkTableInstance.getDefault()
                .getTable("SmartDashboard")
                .getDoubleTopic(entryName)
                .getEntry(defaultValue);

        entry.set(defaultValue);
        return entry;
    }


    public static BooleanEntry createBooleanEntry(String entryName, boolean defaultValue) {
        var entry = edu.wpi.first.networktables.NetworkTableInstance.getDefault()
                .getTable("SmartDashboard")
                .getBooleanTopic(entryName)
                .getEntry(defaultValue);
        
        entry.set(defaultValue);
        return entry;
    }
}
