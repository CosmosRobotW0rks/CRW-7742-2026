package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.auto.autoCommands.ApproachPoseCommand;
import frc.robot.auto.autoCommands.ApproachPoseCommand.ApproachPoseConfiguration;
import frc.robot.subsystems.drivetrain.swerve.SwerveSubsystem;
import frc.robot.utils.FieldUtils;

public class AutoHelper {

    public static Command GetClimbCommand(SwerveSubsystem swerve) {
        final Pose2d wp1 = new Pose2d(1.68, 3.744, Rotation2d.kZero);
        final Pose2d wp2 = new Pose2d(0.951, 3.744, Rotation2d.kZero);

        final ApproachPoseConfiguration config1 = ApproachPoseConfiguration.fromPose(wp1);
        final ApproachPoseConfiguration config2 = ApproachPoseConfiguration.fromPose(wp2).withMaxSpeed(0.5);

        ApproachPoseCommand cmd1 = new ApproachPoseCommand(swerve, config1);
        ApproachPoseCommand cmd2 = new ApproachPoseCommand(swerve, config2);

        return cmd1.andThen(cmd2);
    }

    public static double GetShooterDistanceToHub(Pose2d robotPose) {

        Translation2d hubTranslation = FieldUtils.GetAllianceBasedHubCenter();
        Translation2d shooterOffset = Constants.ShooterConstants.ShooterOffset;

        Translation2d shooterFieldPosition = robotPose.getTranslation().plus(
                shooterOffset.rotateBy(robotPose.getRotation()));

        return shooterFieldPosition.getDistance(hubTranslation);
    }

    public static Rotation2d GetRobotAngleToHub(Translation2d robotPos) {

        Translation2d hubCenter = FieldUtils.GetAllianceBasedHubCenter();

        Translation2d diff = hubCenter.minus(robotPos);

        return new Rotation2d(diff.getX(), diff.getY());
    }
}
