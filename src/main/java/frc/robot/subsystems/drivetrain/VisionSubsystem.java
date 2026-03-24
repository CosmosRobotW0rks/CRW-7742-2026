package frc.robot.subsystems.drivetrain;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.swerve.SwerveSubsystem;

public class VisionSubsystem extends SubsystemBase {

    private static final String MAX_DISTANCE_KEY = "VISION/TUNE/MAX_TAG_DISTANCE_M";
    private static final String XY_STD_DEV_COEFF_KEY = "VISION/TUNE/XY_STD_DEV_COEFF";
    private static final String THETA_STD_DEV_COEFF_KEY = "VISION/TUNE/THETA_STD_DEV_COEFF";

    private static final double DEFAULT_MAX_TAG_DISTANCE_METERS = 4.0;
    private static final double DEFAULT_XY_STD_DEV_COEFF = 0.1;
    private static final double DEFAULT_THETA_STD_DEV_COEFF = 0.2;

    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltAndymark);

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

    private volatile double maxAcceptedTagDistanceMeters = DEFAULT_MAX_TAG_DISTANCE_METERS;
    private volatile double xyStdDevCoeff = DEFAULT_XY_STD_DEV_COEFF;
    private volatile double thetaStdDevCoeff = DEFAULT_THETA_STD_DEV_COEFF;

    private SwerveSubsystem swerveDt;

    public VisionSubsystem(SwerveSubsystem swerveDt) {
        this.swerveDt = swerveDt;

        SmartDashboard.setDefaultNumber(MAX_DISTANCE_KEY, DEFAULT_MAX_TAG_DISTANCE_METERS);
        SmartDashboard.setDefaultNumber(XY_STD_DEV_COEFF_KEY, DEFAULT_XY_STD_DEV_COEFF);
        SmartDashboard.setDefaultNumber(THETA_STD_DEV_COEFF_KEY, DEFAULT_THETA_STD_DEV_COEFF);
    }

    long sucC = 0;
    long totalC = 0;

    @Override
    public void periodic() {
        updateTunables();

        handleCamera(cameraFw, fwEstimator);
        handleCamera(cameraAngled, angledEstimator);

        SmartDashboard.putNumber("VISION/SUCC", sucC);
        SmartDashboard.putNumber("VISION/TOTALC", totalC);
    }

    void handleCamera(PhotonCamera camera, PhotonPoseEstimator estimator) {
        var results = camera.getAllUnreadResults();

        if (results.size() == 0)
            return;

        var result = results.get(results.size() - 1);

        Optional<EstimatedRobotPose> visionEst = estimator.update(result);

        if (!visionEst.isEmpty()) {
            var estimatedRobotPose = visionEst.get();
            double tagDistanceMeters = getClosestTagDistanceMeters(estimatedRobotPose);

            if (Double.isFinite(tagDistanceMeters) && tagDistanceMeters <= maxAcceptedTagDistanceMeters) {
                Pose3d p3d = estimatedRobotPose.estimatedPose;
                Pose2d p2d = p3d.toPose2d();
                Matrix<N3, N1> dynamicStdDevs = calculateDynamicStdDevs(tagDistanceMeters);

                swerveDt.addVisionMeasurement(p2d, estimatedRobotPose.timestampSeconds, dynamicStdDevs);

                SmartDashboard.putNumberArray("VISION/T3D_" + camera.getName(),
                        new Double[] { p3d.getX(), p3d.getY(), p3d.getZ() });
                SmartDashboard.putNumber("VISION/TAG_DISTANCE_" + camera.getName(), tagDistanceMeters);

                sucC++;
            }
        }

        totalC++;

    }

    private void updateTunables() {
        maxAcceptedTagDistanceMeters = SmartDashboard.getNumber(MAX_DISTANCE_KEY, maxAcceptedTagDistanceMeters);
        xyStdDevCoeff = SmartDashboard.getNumber(XY_STD_DEV_COEFF_KEY, xyStdDevCoeff);
        thetaStdDevCoeff = SmartDashboard.getNumber(THETA_STD_DEV_COEFF_KEY, thetaStdDevCoeff);
    }

    private double getClosestTagDistanceMeters(EstimatedRobotPose estimatedRobotPose) {
        return estimatedRobotPose.targetsUsed.stream()
                .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
                .min()
                .orElse(Double.NaN);
    }

    private Matrix<N3, N1> calculateDynamicStdDevs(double tagDistanceMeters) {
        double distanceSquared = tagDistanceMeters * tagDistanceMeters;
        double xyStdDev = xyStdDevCoeff * distanceSquared;
        double thetaStdDev = thetaStdDevCoeff * distanceSquared;

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }
}
