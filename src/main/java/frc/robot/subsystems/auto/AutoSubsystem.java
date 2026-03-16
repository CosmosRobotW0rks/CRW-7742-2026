package frc.robot.subsystems.auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.auto.autoCommands.ApproachPoseCommand;
import frc.robot.subsystems.auto.autoCommands.ApproachPoseCommand.ApproachPoseConfiguration;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drivetrain.swerve.SwerveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.AllianceUtils;
import frc.robot.utils.EntryUtils;

public class AutoSubsystem extends SubsystemBase {

    BooleanEntry entry_autoEnabled = EntryUtils.createBooleanEntry("AutoConfig/Auto Enabled", true);

    SendableChooser<String> chooser_startPos = EntryUtils.createSendableChooser("AutoConfig/Start Pos",
     "Left Trench", "Left Bump", "Middle", "Right Bump", "Right Trench");


    SwerveSubsystem swerve;
    ShooterSubsystem shooter;
    ClimbSubsystem climb;

    public AutoSubsystem(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem, ClimbSubsystem climbSubsystem) {
        this.swerve = swerveSubsystem;
        this.shooter = shooterSubsystem;
        this.climb = climbSubsystem;
    }

    public Pose2d getStartPos()
    {
        String selected = chooser_startPos.getSelected();
        if (selected == null) selected = "Middle";

        Pose2d pose = switch (selected) {
            case "Left Trench" -> AllianceUtils.FlipVertically(AutoConstants.StartPose_RightTrench);
            case "Left Bump" -> AllianceUtils.FlipVertically(AutoConstants.StartPose_RightBump);
            case "Middle" -> AutoConstants.StartPose_Middle;
            case "Right Bump" -> AutoConstants.StartPose_RightBump;
            case "Right Trench" -> AutoConstants.StartPose_RightTrench;
            default -> AutoConstants.StartPose_Middle;
        };

        return AllianceUtils.FlipIfRed(pose);
    }

    public Command getAutoCommand() {
        if(!entry_autoEnabled.get()) return Commands.none();

        Pose2d startPose = getStartPos();


        Pose2d wp1 = startPose.transformBy(new Transform2d(-2, 0, Rotation2d.kZero));
        wp1 = AllianceUtils.FlipIfRed(wp1);

        ApproachPoseConfiguration approachConfig = ApproachPoseConfiguration.fromPose(wp1).withMaxSpeed(1);

        Command cmd1_taxi = new ApproachPoseCommand(swerve, approachConfig);

        Command cmd2_1_prepShooter = shooter.prepareShooterCommand();

        Command cmd2_2_approach = AutoHelper.GetApproachHubCommand(swerve, () -> swerve.getRobotPose(), () -> 0.35);

        Command cmd3_prepAndShoot = shooter.prepareAndShootCommand(true);

        return cmd1_taxi.andThen(cmd2_1_prepShooter.raceWith(cmd2_2_approach)).andThen(cmd3_prepAndShoot);
    }

    @Override
    public void periodic() {

        if (DriverStation.isDisabled()) {
            swerve.SetStartPose(getStartPos());
        }
    }

}
