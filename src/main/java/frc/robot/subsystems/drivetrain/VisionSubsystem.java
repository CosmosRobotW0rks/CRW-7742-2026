package frc.robot.subsystems.drivetrain;

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
import frc.robot.subsystems.drivetrain.swerve.SwerveSubsystem;

public class VisionSubsystem extends SubsystemBase {

    PhotonCamera cameraFw = new PhotonCamera("fwcam");
    PhotonCamera cameraAngled = new PhotonCamera("angledcam");

    public static final Transform3d kRobotToFwCam = new Transform3d(new Translation3d(-0.0885, 0.269, 0.4965),
            new Rotation3d(0, 0, 0));

    public static final Transform3d kRobotToAngledCam = new Transform3d(new Translation3d(0.04, -0.268555, 0.495),
            new Rotation3d(0, Math.toRadians(-35), 0));

    private PhotonPoseEstimator fwEstimator = new PhotonPoseEstimator(kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToFwCam);
    private PhotonPoseEstimator angledEstimator = new PhotonPoseEstimator(kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToAngledCam);

            
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    
    private SwerveSubsystem swerveDt;
   
    public VisionSubsystem(SwerveSubsystem swerveDt) {
        this.swerveDt = swerveDt;
    }

    long sucC = 0;
    long totalC = 0;

    @Override
    public void periodic() {
        
        handleCamera(cameraFw, fwEstimator);
        handleCamera(cameraAngled, angledEstimator);

        SmartDashboard.putNumber("VISION/SUCC", sucC);
        SmartDashboard.putNumber("VISION/TOTALC", totalC);
    }

    void handleCamera(PhotonCamera camera, PhotonPoseEstimator estimator)
    {
        var results = camera.getAllUnreadResults();

        if (results.size() == 0)
            return;

        var result = results.get(results.size() - 1);


        Optional<EstimatedRobotPose> visionEst = estimator.update(result);

        if (!visionEst.isEmpty()) {

            Pose3d p3d = visionEst.get().estimatedPose;
            Pose2d p2d = p3d.toPose2d();

            Pose2d rotatedP2d = new Pose2d(p2d.getX(), p2d.getY(), p2d.getRotation().rotateBy(Rotation2d.kZero));
            swerveDt.addVisionMeasurement(rotatedP2d, visionEst.get().timestampSeconds);

            SmartDashboard.putNumberArray("VISION/T3D_" + camera.getName(), new Double[] { p3d.getX(), p3d.getY(), p3d.getZ() });

            sucC++;
        }

        totalC++;

    }
}
