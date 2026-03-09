package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class FieldUtils {
    public static Translation2d GetAllianceBasedHubCenter()
    {
        Translation2d blueHubCenter = Constants.FieldConstants.HubCenter;
        
        return AllianceUtils.FlipIfRed(blueHubCenter);
    }

    public static boolean IsInSelfAllianceHalf(Translation2d position)
    {
        double blueEndX = 4;
        double redStartX = 12;
        
        if(AllianceUtils.isRedAlliance())
        {
            return position.getX() > redStartX;
        }
        else
        {
            return position.getX() < blueEndX;
        }
    }
}
