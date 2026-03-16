package frc.robot.subsystems.auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.auto.autoCommands.ApproachPoseCommand;
import frc.robot.subsystems.auto.autoCommands.ApproachPoseCommand.ApproachPoseConfiguration;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drivetrain.swerve.SwerveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.AllianceUtils;
import frc.robot.utils.EntryUtils;

public class AutoSubsystem extends SubsystemBase {

    SwerveSubsystem swerve;
    ShooterSubsystem shooter;
    ClimbSubsystem climb;

    public AutoSubsystem(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem, ClimbSubsystem climbSubsystem) {
        this.swerve = swerveSubsystem;
        this.shooter = shooterSubsystem;
        this.climb = climbSubsystem;
    }

    public Command getAutoCommand() {
        Pose2d wp1 = DriveConstants.defaultStartPose.transformBy(new Transform2d(-2, 0, Rotation2d.kZero));
        wp1 = AllianceUtils.FlipIfRed(wp1);

        ApproachPoseConfiguration approachConfig = ApproachPoseConfiguration.fromPose(wp1).withMaxSpeed(1);

        Command cmd1_taxi = new ApproachPoseCommand(swerve, approachConfig);

        Command cmd2_1_prepShooter = shooter.prepareShooterCommand();

        Command cmd2_2_approach = AutoHelper.GetApproachHubCommand(swerve, () -> swerve.getRobotPose(), () -> 0.35);

        Command cmd3_prepAndShoot = shooter.prepareAndShootCommand(true);

        return cmd1_taxi.andThen(cmd2_1_prepShooter.raceWith(cmd2_2_approach)).andThen(cmd3_prepAndShoot);
    }

}
