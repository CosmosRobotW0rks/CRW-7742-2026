package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

            
    private final static StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("EstimatedHubShootPose", Pose2d.struct).publish();

    public static Command GetApproachHubCommand(SwerveSubsystem swerve, Pose2d robotPose, Supplier<Double> speedCoeff) {
        final Translation2d hubPos = FieldUtils.GetAllianceBasedHubCenter();
        final double shootingDistance = Constants.ShooterConstants.ShootingDistanceM;

        if(!FieldUtils.IsInSelfAllianceHalf(robotPose.getTranslation())) return Commands.none();

        Translation2d hubToRobot = robotPose.getTranslation().minus(hubPos);
        Translation2d direction = hubToRobot.div(hubToRobot.getNorm());

        Translation2d targetTranslation = hubPos.plus(direction.times(shootingDistance));
        Rotation2d targetRotation = GetRobotAngleToHub(targetTranslation);

        Pose2d targetPose = new Pose2d(targetTranslation, targetRotation);


        posePublisher.set(targetPose);
        
        ApproachPoseConfiguration config = ApproachPoseConfiguration.fromPose(targetPose)
            .withMaxSpeed(() -> speedCoeff.get() * Constants.AutoConstants.ApproachPose_Default_maxSpeedMPS);

        //return Commands.none();
        return new ApproachPoseCommand(swerve, config);
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
