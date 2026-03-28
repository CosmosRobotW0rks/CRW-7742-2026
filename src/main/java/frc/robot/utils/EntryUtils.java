package frc.robot.utils;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    public static SendableChooser<String> createSendableChooser(String name, String... options) {

        SendableChooser<String> chooser = new SendableChooser<String>();

        for (String option : options) {
            chooser.addOption(option, option);
        }

        chooser.setDefaultOption(options[0], options[0]);

        SmartDashboard.putData(name, chooser);
        
        return chooser;
    }
}
