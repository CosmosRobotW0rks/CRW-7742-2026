package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.autoCommands.ApproachPoseCommand;
import frc.robot.auto.autoCommands.ApproachPoseCommand.ApproachPoseConfiguration;
import frc.robot.subsystems.drivetrain.swerve.SwerveSubsystem;

public class AutoHelper {

    public static Command GetClimbCommand(SwerveSubsystem swerve)
    {
        final Pose2d wp1 = new Pose2d(1.68, 3.744, Rotation2d.kZero);
        final Pose2d wp2 = new Pose2d(0.951, 3.744, Rotation2d.kZero);

        final ApproachPoseConfiguration config1 = new ApproachPoseConfiguration(wp1);
        final ApproachPoseConfiguration config2 = new ApproachPoseConfiguration(wp2).withMaxSpeed(0.5);

        ApproachPoseCommand cmd1 = new ApproachPoseCommand(swerve, config1);
        ApproachPoseCommand cmd2 = new ApproachPoseCommand(swerve, config2);


        return cmd1.andThen(cmd2);
    }
}
