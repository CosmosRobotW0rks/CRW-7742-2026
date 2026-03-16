package frc.robot.subsystems.auto;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.utils.Logging;

public class AutoSubsystem extends SubsystemBase {

    private final double maxSpeed_TaxiTrans = 2.0;
    private final Rotation2d maxSpeed_TaxiRot = Rotation2d.fromRotations(0.3);
    private final double speedCoeff_approachHub = 0.75;

    BooleanEntry entry_autoEnabled = EntryUtils.createBooleanEntry("AutoConfig/Auto Enabled", true);
    BooleanEntry entry_alignAndShootEnabled = EntryUtils.createBooleanEntry("AutoConfig/Align and Shoot Enabled", true);
    BooleanEntry entry_climbEnabled = EntryUtils.createBooleanEntry("AutoConfig/Climb Enabled", true);

    SendableChooser<String> chooser_startPos = EntryUtils.createSendableChooser("AutoConfig/Start Pos",
     "Left Trench", "Left Bump", "Right Bump", "Right Trench");

    SendableChooser<String> chooser_climbSide = EntryUtils.createSendableChooser("AutoConfig/Climb Side",
     "Left");

    DoubleEntry entry_taxiX = EntryUtils.createDoubleEntry("AutoConfig/Taxi X", 2.0);
    DoubleEntry entry_climbTravelX = EntryUtils.createDoubleEntry("AutoConfig/Climb Travel X", 1.6);

    DoubleEntry entry_climbY = EntryUtils.createDoubleEntry("AutoConfig/Climb Y", 4.629);
    DoubleEntry entry_climbBackwardX = EntryUtils.createDoubleEntry("AutoConfig/Climb Backward X", 0.55);
    DoubleEntry entry_climbForwardX = EntryUtils.createDoubleEntry("AutoConfig/Climb Forward X", 0.9);

    SwerveSubsystem swerve;
    ShooterSubsystem shooter;
    ClimbSubsystem climb;

    public AutoSubsystem(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem, ClimbSubsystem climbSubsystem) {
        this.swerve = swerveSubsystem;
        this.shooter = shooterSubsystem;
        this.climb = climbSubsystem;
    }

    public Pose2d getBlueStartPos()
    {
        String selected = chooser_startPos.getSelected();
        if (selected == null) selected = "Middle";

        Pose2d pose = switch (selected) {
            case "Left Trench" -> AllianceUtils.FlipVertically(AutoConstants.StartPose_RightTrench, false);
            case "Left Bump" -> AllianceUtils.FlipVertically(AutoConstants.StartPose_RightBump, false);
            case "Middle" -> AutoConstants.StartPose_Middle;
            case "Right Bump" -> AutoConstants.StartPose_RightBump;
            case "Right Trench" -> AutoConstants.StartPose_RightTrench;
            default -> AutoConstants.StartPose_Middle;
        };

        return pose;
    }

    public Command getAutoCommand() {

        if (!entry_autoEnabled.get())
            return Commands.none();

        boolean en_alignAndShoot = entry_alignAndShootEnabled.get();
        boolean en_climb = entry_climbEnabled.get();

        Command base = Commands.none();
        base = addTaxi(base);

        if (en_alignAndShoot) 
            base = addApproachAndShoot(base);

        if (en_climb)
            base = addClimb(base);

        return base;
    }

    private Command alertCmd(String msg) {
        return Commands.runOnce(() -> Logging.infoMsg("CMD", msg));
    }
    

    private Command addTaxi(Command cmd)
    {
        Pose2d startPose = getBlueStartPos();

        Translation2d wp1Translation = new Translation2d(entry_taxiX.get(), startPose.getY());
        Rotation2d wp1Rotation = AutoHelper.GetRobotAngleToBlueHub(wp1Translation);

        Pose2d wp1 = new Pose2d(wp1Translation, wp1Rotation);
        wp1 = AllianceUtils.FlipIfRed(wp1);

        ApproachPoseConfiguration approachConfig = ApproachPoseConfiguration.fromPose(wp1).withAngularVelocity(maxSpeed_TaxiRot).withMaxSpeed(maxSpeed_TaxiTrans);

        Command cmdTaxi = new ApproachPoseCommand(swerve, approachConfig);

        return cmd.andThen(cmdTaxi);
    }

    private Command addApproachAndShoot(Command cmd)
    {
        Command cmd1_prepShooter = shooter.prepareShooterCommand();

        Command cmd2_approach = AutoHelper.GetApproachHubCommand(swerve, () -> swerve.getRobotPose(), () -> speedCoeff_approachHub);

        Command cmd3_prepAndShoot = shooter.prepareAndShootCommand(true);

        Command cmd4_sleepShooter = shooter.sleepShooterCommand();

        return cmd.andThen(cmd1_prepShooter.raceWith(cmd2_approach)).andThen(cmd3_prepAndShoot.withTimeout(5)).andThen(cmd4_sleepShooter); // TODO: Variable time
    }

    private Command addClimb(Command cmd)
    {
        cmd = cmd.andThen(climb.Down());
        cmd = addClimbTaxi_X_IfRight(cmd);
        cmd = addClimbTaxi(cmd);
        cmd = addClimbBackward(cmd);
        cmd = cmd.andThen(climb.Up(), Commands.waitSeconds(0.35));
        cmd = addClimbForward(cmd);
        cmd = cmd.andThen(climb.Down(), Commands.waitSeconds(0.35));


        return cmd;
    }

    private Command addClimbTaxi_X_IfRight(Command cmd)
    {
        Command xMatch = Commands.defer(() -> {
            Pose2d currentBluePose = AllianceUtils.FlipIfRed(swerve.getRobotPose());

            if(currentBluePose.getY() > 4.25) return Commands.none(); // On the left side of the tower


            double targetX = entry_climbTravelX.get();

            Pose2d targetPose = new Pose2d(new Translation2d(targetX, currentBluePose.getY()), Rotation2d.k180deg);
            targetPose = AllianceUtils.FlipIfRed(targetPose);

            ApproachPoseConfiguration config = ApproachPoseConfiguration.fromPose(targetPose)
                    .withMaxSpeed(1.5)
                    .withAngularVelocity(Rotation2d.fromRotations(0.3));

            return new ApproachPoseCommand(swerve, config);
        }, Set.of());

        return cmd.andThen(xMatch);
    }

    private Command addClimbTaxi(Command cmd)
    {
        Pose2d targetPose = new Pose2d(entry_climbTravelX.get(), entry_climbY.get(), Rotation2d.k180deg);
        targetPose = AllianceUtils.FlipIfRed(targetPose);

        ApproachPoseConfiguration config = ApproachPoseConfiguration.fromPose(targetPose)
                .withMaxSpeed(1.5)
                .withAngularVelocity(Rotation2d.fromRotations(0.3));
        
        Command cmdTaxi = new ApproachPoseCommand(swerve, config);

        return cmd.andThen(cmdTaxi);
    }

    private Command addClimbBackward(Command cmd)
    {
        Pose2d targetPose = new Pose2d(entry_climbBackwardX.get(), entry_climbY.get(), Rotation2d.k180deg);
        targetPose = AllianceUtils.FlipIfRed(targetPose);

        ApproachPoseConfiguration config = ApproachPoseConfiguration.fromPose(targetPose)
                .withMaxSpeed(0.5)
                .withAngularVelocity(Rotation2d.fromRotations(0.1));
        
        Command cmdTaxi = new ApproachPoseCommand(swerve, config);

        return cmd.andThen(cmdTaxi);
    }

    private Command addClimbForward(Command cmd)
    {
        Pose2d targetPose = new Pose2d(entry_climbForwardX.get(), entry_climbY.get(), Rotation2d.k180deg);
        targetPose = AllianceUtils.FlipIfRed(targetPose);

        ApproachPoseConfiguration config = ApproachPoseConfiguration.fromPose(targetPose)
                .withMaxSpeed(0.3)
                .withAngularVelocity(Rotation2d.fromRotations(0.1));
        
        Command cmdTaxi = new ApproachPoseCommand(swerve, config);

        return cmd.andThen(cmdTaxi);
    }



    @Override
    public void periodic() {

        if (DriverStation.isDisabled()) {

            Pose2d blueStartPose = getBlueStartPos();
            Pose2d startPose = AllianceUtils.FlipIfRed(blueStartPose);
            swerve.SetStartPose(startPose);
        }
    }

}
