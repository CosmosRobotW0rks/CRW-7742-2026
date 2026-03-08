package frc.robot.auto.autoCommands;

import java.util.function.Supplier;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drivetrain.swerve.SwerveSubsystem;

public class ApproachPoseCommand extends Command {

    private final SwerveSubsystem swerve;

    private final PIDController pid_x;
    private final PIDController pid_y;
    private final ProfiledPIDController pid_z;

    private final ApproachPoseConfiguration config;

    public ApproachPoseCommand(SwerveSubsystem swerve, ApproachPoseConfiguration config) {
        this.swerve = swerve;
        this.config = config;

        PIDConstants translationPID = AutoConstants.ApproachPose_Translation_PID;
        PIDConstants rotationPID = AutoConstants.ApproachPose_Rotation_PID;

        pid_x = new PIDController(translationPID.kP, translationPID.kI, translationPID.kD);
        pid_y = new PIDController(translationPID.kP, translationPID.kI, translationPID.kD);

        pid_z = new ProfiledPIDController(rotationPID.kP, rotationPID.kI, rotationPID.kD,
                new TrapezoidProfile.Constraints(config.maxAngularVelocity.getRadians(),
                        config.maxAngularAccel.getRadians()));

        pid_z.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        Pose2d current = swerve.getRobotPose();

        pid_x.reset();
        pid_y.reset();
        pid_z.reset(current.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getRobotPose();
        Pose2d targetPose = config.poseSupplier.get();

        double xSpeed = config.translate ? pid_x.calculate(currentPose.getX(), targetPose.getX()) : 0;

        double ySpeed = config.translate ? pid_y.calculate(currentPose.getY(), targetPose.getY()) : 0;

        double zSpeed = config.rotate ? -pid_z.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians()) : 0;

        double mag = Math.hypot(xSpeed, ySpeed);

        if (mag > config.maxSpeedMPS) {
            xSpeed = (xSpeed / mag) * config.maxSpeedMPS;
            ySpeed = (ySpeed / mag) * config.maxSpeedMPS;
        }

        ChassisSpeeds cs = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, currentPose.getRotation());

        swerve.setTargetRobotRelativeSpeeds(cs);

        SmartDashboard.putNumberArray("Auto/ApproachPoseCmd/TargetSpeeds",new double[]{xSpeed, ySpeed, zSpeed});
    }

    @Override
    public void end(boolean interrupted) {
        swerve.Stop();
    }

    @Override
    public boolean isFinished() {

        if(!config.finishable) return false;

        Pose2d targetPose = config.poseSupplier.get();
        Pose2d error = swerve.getRobotPose().relativeTo(targetPose);

        SmartDashboard.putNumberArray("Auto/ApproachPoseCmd/PoseError", new double[]{error.getX(), error.getY(), error.getRotation().getRadians()});


        return Math.abs(error.getX()) < config.xErrorThresholdM &&
                Math.abs(error.getY()) < config.yErrorThresholdM &&
                Math.abs(error.getRotation().getRadians()) < config.rotErrorThreshold.getRadians();
    }

    public static class ApproachPoseConfiguration {

        private final Supplier<Pose2d> poseSupplier;

        private boolean translate = true;
        private boolean rotate = true;

        private boolean finishable = true;

        private double maxSpeedMPS = AutoConstants.ApproachPose_Default_maxSpeedMPS;
        private Rotation2d maxAngularVelocity = AutoConstants.ApproachPose_Default_maxAngVelocity;
        private Rotation2d maxAngularAccel = AutoConstants.ApproachPose_Default_maxAngAccel;

        private double xErrorThresholdM = AutoConstants.ApproachPose_Default_translationErrorThresholdM;
        private double yErrorThresholdM = AutoConstants.ApproachPose_Default_translationErrorThresholdM;
        private Rotation2d rotErrorThreshold = AutoConstants.ApproachPose_Default_rotationErrorThreshold;

        // CTORS

        private ApproachPoseConfiguration(Pose2d pose) {
            poseSupplier = () -> pose;
        }

        private ApproachPoseConfiguration(Supplier<Pose2d> poseSup) {
            poseSupplier = poseSup;
        }

        // STATIC CTORS
        public static ApproachPoseConfiguration fromPose(Pose2d pose)
        {
            return new ApproachPoseConfiguration(pose);
        }

        public static ApproachPoseConfiguration fromPose(Supplier<Pose2d> pose)
        {
            return new ApproachPoseConfiguration(pose);
        }

        public static ApproachPoseConfiguration fromRotation(Rotation2d r2d) {
            return new ApproachPoseConfiguration(new Pose2d(Translation2d.kZero, r2d)).toggleMovement(false, true);
        }

        public static ApproachPoseConfiguration fromRotation(Supplier<Rotation2d> r2d) {
            return new ApproachPoseConfiguration(() -> new Pose2d(Translation2d.kZero, r2d.get())).toggleMovement(false, true);
        }

        public static ApproachPoseConfiguration fromTranslation(Translation2d t2d) {
            return new ApproachPoseConfiguration(new Pose2d(t2d, Rotation2d.kZero)).toggleMovement(true, false);
        }

        public static ApproachPoseConfiguration fromTranslation(Supplier<Translation2d> t2d) {
            return new ApproachPoseConfiguration(() -> new Pose2d(t2d.get(), Rotation2d.kZero)).toggleMovement(true, false);
        }

        // MODIFIERS

        public ApproachPoseConfiguration withMaxSpeed(double maxSpeed) {
            this.maxSpeedMPS = maxSpeed;
            return this;
        }

        public ApproachPoseConfiguration withAngularVA(Rotation2d maxAngularVelocity, Rotation2d maxAngularAccel) {
            this.maxAngularVelocity = maxAngularVelocity;
            this.maxAngularAccel = maxAngularAccel;
            return this;
        }

        public ApproachPoseConfiguration withErrorThreshold(double perAxisErrorThresholdM,
                Rotation2d rotErrorThreshold) {
            this.xErrorThresholdM = perAxisErrorThresholdM;
            this.yErrorThresholdM = perAxisErrorThresholdM;
            this.rotErrorThreshold = rotErrorThreshold;
            return this;
        }

        public ApproachPoseConfiguration withErrorThreshold(double xErrorThresholdM, double yErrorThresholdM,
                Rotation2d rotErrorThreshold) {
            this.xErrorThresholdM = xErrorThresholdM;
            this.yErrorThresholdM = yErrorThresholdM;
            this.rotErrorThreshold = rotErrorThreshold;
            return this;
        }

        public ApproachPoseConfiguration toggleMovement(boolean translate, boolean rotate)
        {
            this.translate = translate;
            this.rotate = rotate;
            return this;
        }

        public ApproachPoseConfiguration toggleFinishable(boolean finishable)
        {
            this.finishable = finishable;
            return this;
        }
    }

}