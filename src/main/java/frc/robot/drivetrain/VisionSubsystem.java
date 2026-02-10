package frc.robot.drivetrain;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivetrain.swerve.SwerveSubsystem;

// TODO: Implement VisionSubsystem
public class VisionSubsystem extends SubsystemBase {

    PhotonCamera camera = new PhotonCamera("maincam");

    // TODO: Use 2026 field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.325, 0.0, 0),
            new Rotation3d(0, -65, 0));

    private PhotonPoseEstimator estimator = new PhotonPoseEstimator(kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);

    private SwerveSubsystem swerveDt;

    public VisionSubsystem(SwerveSubsystem swerveDt) {
        this.swerveDt = swerveDt;
    }

    long sucC = 0;
    long totalC = 0;

    @Override
    public void periodic() {
        var results = camera.getAllUnreadResults();

        if (results.size() == 0)
            return;

        var result = results.get(results.size() - 1);

        Optional<EstimatedRobotPose> visionEst = estimator.update(result);

        if (!visionEst.isEmpty()) {

            Pose3d p3d = visionEst.get().estimatedPose;
            Pose2d p2d = p3d.toPose2d();

            // TODO: Check if rotation is required
            Pose2d rotatedP2d = new Pose2d(p2d.getX(), p2d.getY(), p2d.getRotation().rotateBy(Rotation2d.k180deg));
            swerveDt.addVisionMeasurement(rotatedP2d, visionEst.get().timestampSeconds);

            SmartDashboard.putNumberArray("PHOTONTEST/T3D", new Double[] { p3d.getX(), p3d.getY(), p3d.getZ() });

            sucC++;
        }

        totalC++;

        SmartDashboard.putNumber("PHOTONTEST/SUCC", sucC);
        SmartDashboard.putNumber("PHOTONTEST/TOTALC", totalC);
    }

    void a() {

        var results = camera.getAllUnreadResults();

        if (results.size() == 0)
            return;

        var res = results.get(results.size() - 1);

        if (!res.hasTargets())
            return;

        var target = res.getBestTarget();

        Transform3d t3d = target.getBestCameraToTarget();

        SmartDashboard.putNumber("PHOTONTEST/T3D_X", t3d.getX());
        SmartDashboard.putNumber("PHOTONTEST/T3D_Y", t3d.getY());
        SmartDashboard.putNumber("PHOTONTEST/T3D_Z", t3d.getZ());
    }
}
