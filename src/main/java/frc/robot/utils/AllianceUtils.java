package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public final class AllianceUtils {

    final static double FIELD_WIDTH = Units.inchesToMeters(651.2);
    final static double FIELD_HEIGHT = Units.inchesToMeters(317.7);

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public static Pose2d Flip(Pose2d pose)
    {
        Pose2d newPose = new Pose2d(FIELD_WIDTH - pose.getX(), FIELD_HEIGHT-pose.getY(), pose.getRotation().rotateBy(Rotation2d.k180deg));
        return newPose;
    }

    public static Translation2d Flip(Translation2d translation)
    {
        Translation2d newTranslation = new Translation2d(FIELD_WIDTH - translation.getX(), FIELD_HEIGHT-translation.getY());
        return newTranslation;
    }

    public static Pose2d FlipIfRed(Pose2d pose)
    {
        return isRedAlliance() ? Flip(pose) : pose;
    }

    public static Translation2d FlipIfRed(Translation2d translation)
    {
        return isRedAlliance() ? Flip(translation) : translation;
    }
}
