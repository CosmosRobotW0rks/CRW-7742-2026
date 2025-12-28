package frc.robot.drivetrain.swerve;

import java.util.Map;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.DriveGearRatioOption;
import frc.robot.drivetrain.swerve.common.SwerveModuleConfiguration;
import frc.robot.drivetrain.swerve.common.SwerveModuleLocation;
import frc.robot.drivetrain.swerve.common.SwerveMotorConfig;

public class SwerveSubsystem extends SubsystemBase {

    // For testing individual modules
    private final SwerveModuleLocation isolatedModule = SwerveModuleLocation.FRONT_LEFT;

    // TODO: Shuffleboard selection
    private final DriveGearRatioOption driveGearRatioOption = DriveGearRatioOption.R2;

    // External sensors
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    // Swerve related

    private Map<SwerveModuleLocation, SwerveModule> modules = new java.util.EnumMap<>(SwerveModuleLocation.class);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveConstants.TrackWidthM / 2, SwerveConstants.TrackWidthM / 2), // Front Left
            new Translation2d(SwerveConstants.TrackWidthM / 2, -SwerveConstants.TrackWidthM / 2), // Front Right
            new Translation2d(-SwerveConstants.TrackWidthM / 2, SwerveConstants.TrackWidthM / 2), // Back Left
            new Translation2d(-SwerveConstants.TrackWidthM / 2, -SwerveConstants.TrackWidthM / 2) // Back Right
    );

    ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds(0, 0, 0);

    // Constructor
    public SwerveSubsystem() {
        boolean moduleInitSuc = initModules();
        if (!moduleInitSuc) {
            DriverStation.reportError("Failed to initialize Swerve Subsystem", false);
        }
    }

    // Actions
    public Command Stop() {
        return runOnce(() -> {
            targetChassisSpeeds = new ChassisSpeeds(0, 0, 0);
            modules.values().forEach(module -> module.stop());
        });
    }

    // Getters
    public Rotation2d getRobotHeading() {
        // TODO: Make sure returned value is CCW positive
        return Rotation2d.fromDegrees(360.0 - gyro.getFusedHeading() + DriveConstants.RobotStartAngle.getDegrees());
    }

    public ChassisSpeeds getTargetChassisSpeeds() {
        return targetChassisSpeeds;
    }

    public ChassisSpeeds getTargetFieldOrientedSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                targetChassisSpeeds.vxMetersPerSecond,
                targetChassisSpeeds.vyMetersPerSecond,
                targetChassisSpeeds.omegaRadiansPerSecond,
                gyro.getRotation2d());
    }

    // Setters
    public void setTargetSpeeds(ChassisSpeeds cs) {
        targetChassisSpeeds = cs;
    }

    public void setTargetFieldOrientedSpeeds(ChassisSpeeds cs) {
        var robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                cs.vxMetersPerSecond,
                cs.vyMetersPerSecond,
                cs.omegaRadiansPerSecond,
                gyro.getRotation2d());

        setTargetSpeeds(robotRelativeSpeeds);
    }

    // Periodic (20ms!!)
    @Override
    public void periodic() {
        applyChassisSpeeds(targetChassisSpeeds);
    }

    // Private methods

    private void applyChassisSpeeds(ChassisSpeeds cs) {
        var states = kinematics.toSwerveModuleStates(cs);

        for (SwerveModuleLocation location : SwerveModuleLocation.values()) {
            if (isolatedModule != null && location != isolatedModule)
                continue;

            SwerveModule module = modules.get(location);
            SwerveModuleState state = states[location.ordinal()];

            state.speedMetersPerSecond *= state.angle.minus(module.getAngle()).getCos();

            module.SetTargetState(state);
        }
    }

    private boolean initModules() {

        if (isolatedModule != null)
            DriverStation.reportWarning("WARNING!!! SWERVE MODULE ISOLATED: " + isolatedModule.name(), false);

        for (SwerveModuleLocation location : SwerveModuleLocation.values()) {
            if (isolatedModule != null && location != isolatedModule)
                continue;

            boolean suc = initModule(location);

            if (!suc) {
                DriverStation.reportError("Failed to initialize Swerve Module: " + location.name(), false);
                return false;
            }
        }

        return true;
    }

    private boolean initModule(SwerveModuleLocation location) {
        SwerveAngleKraken angleMotor = new SwerveAngleKraken();
        SwerveDriveKraken driveMotor = new SwerveDriveKraken();

        SwerveModule module = new SwerveModule();

        boolean angleMotorInitSuc = angleMotor.init(new SwerveMotorConfig(
                SwerveConstants.AngleCANIDMap.get(location),

                SwerveConstants.GearRatio_Angle,
                SwerveConstants.WheelRadiusM,

                SwerveConstants.AnglePeakVoltage,

                SwerveConstants.AnglePIDV_P,
                SwerveConstants.AnglePIDV_I,
                SwerveConstants.AnglePIDV_D,
                SwerveConstants.AnglePIDV_V));

        if (!angleMotorInitSuc) {
            DriverStation.reportError("Failed to initialize Swerve Angle Motor for module: " + location.name(), false);
            return false;
        }

        boolean driveMotorInitSuc = driveMotor.init(new SwerveMotorConfig(
                SwerveConstants.DriveCANIDMap.get(location),

                SwerveConstants.GearRatioMap_Drive.get(driveGearRatioOption),
                SwerveConstants.WheelRadiusM,

                SwerveConstants.DrivePeakVoltage,

                SwerveConstants.DrivePIDV_P,
                SwerveConstants.DrivePIDV_I,
                SwerveConstants.DrivePIDV_D,
                SwerveConstants.DrivePIDV_V));

        if (!driveMotorInitSuc) {
            DriverStation.reportError("Failed to initialize Swerve Drive Motor for module: " + location.name(), false);
            return false;
        }

        module.init(new SwerveModuleConfiguration(location, angleMotor, driveMotor));

        modules.put(location, module);

        return true;
    }

}
