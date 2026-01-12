package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;

public final class AllianceUtils {
    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
}
