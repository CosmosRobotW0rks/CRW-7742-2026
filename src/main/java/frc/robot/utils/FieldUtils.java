package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class FieldUtils {
    public static Translation2d GetAllianceBasedHubCenter()
    {
        Translation2d blueHubCenter = Constants.FieldConstants.HubCenter;
        
        return AllianceUtils.FlipIfRed(blueHubCenter);
    }
}
