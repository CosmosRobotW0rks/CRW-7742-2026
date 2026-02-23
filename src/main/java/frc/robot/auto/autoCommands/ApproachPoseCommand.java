package frc.robot.auto.autoCommands;

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
    private final Pose2d targetPose;
    private final double maxSpeedMPS;

    private final PIDController pid_x;
    private final PIDController pid_y;
    private final ProfiledPIDController pid_z;

    private final double xErrorThresholdM;
    private final double yErrorThresholdM;
    private final Rotation2d rotErrorThreshold;

    public ApproachPoseCommand(SwerveSubsystem swerve, ApproachPoseConfiguration config) {
        this.swerve = swerve;
        this.targetPose = config.target;
        this.maxSpeedMPS = config.maxSpeedMPS;

        this.xErrorThresholdM = config.xErrorThresholdM;
        this.yErrorThresholdM = config.yErrorThresholdM;
        this.rotErrorThreshold = config.rotErrorThreshold;

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
        Pose2d current = swerve.getRobotPose();

        double xSpeed = pid_x.calculate(current.getX(), targetPose.getX());

        double ySpeed = pid_y.calculate(current.getY(), targetPose.getY());

        double zSpeed = -pid_z.calculate(current.getRotation().getRadians(), targetPose.getRotation().getRadians());

        double mag = Math.hypot(xSpeed, ySpeed);

        if (mag > maxSpeedMPS) {
            xSpeed = (xSpeed / mag) * maxSpeedMPS;
            ySpeed = (ySpeed / mag) * maxSpeedMPS;
        }

        ChassisSpeeds cs = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, current.getRotation());

        swerve.setTargetRobotRelativeSpeeds(cs);

        SmartDashboard.putNumberArray("Auto/ApproachPoseCmd/TargetSpeeds",new double[]{xSpeed, ySpeed, zSpeed});
    }

    @Override
    public void end(boolean interrupted) {
        swerve.Stop();
    }

    @Override
    public boolean isFinished() {
        Pose2d error = swerve.getRobotPose().relativeTo(targetPose);

        SmartDashboard.putNumberArray("Auto/ApproachPoseCmd/PoseError", new double[]{error.getX(), error.getY(), error.getRotation().getRadians()});


        return Math.abs(error.getX()) < xErrorThresholdM &&
                Math.abs(error.getY()) < yErrorThresholdM &&
                Math.abs(error.getRotation().getRadians()) < rotErrorThreshold.getRadians();
    }

    public static class ApproachPoseConfiguration {

        private final Pose2d target;

        private double maxSpeedMPS = AutoConstants.ApproachPose_Default_maxSpeedMPS;
        private Rotation2d maxAngularVelocity = AutoConstants.ApproachPose_Default_maxAngVelocity;
        private Rotation2d maxAngularAccel = AutoConstants.ApproachPose_Default_maxAngAccel;

        private double xErrorThresholdM = AutoConstants.ApproachPose_Default_translationErrorThresholdM;
        private double yErrorThresholdM = AutoConstants.ApproachPose_Default_translationErrorThresholdM;
        private Rotation2d rotErrorThreshold = AutoConstants.ApproachPose_Default_rotationErrorThreshold;

        public ApproachPoseConfiguration(Pose2d pose) {
            target = pose;
        }

        public ApproachPoseConfiguration(Translation2d t2d) {
            target = new Pose2d(t2d, Rotation2d.kZero);
        }

        public ApproachPoseConfiguration(Translation2d t2d, Rotation2d r2d) {
            target = new Pose2d(t2d, r2d);
        }

        public ApproachPoseConfiguration(double x, double y) {
            target = new Pose2d(new Translation2d(x, y), Rotation2d.kZero);
        }

        public ApproachPoseConfiguration(double x, double y, Rotation2d r2d) {
            target = new Pose2d(new Translation2d(x, y), r2d);
        }

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
    }

}