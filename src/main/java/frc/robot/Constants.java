// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.drivetrain.swerve.common.SwerveModuleLocation;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final boolean isDualsense = true;
   
    public static final int DualSenseAxisLeftX = 0;
    public static final int DualSenseAxisLeftY = 1;
    public static final int DualSenseAxisRightX = 2;
  }

  public static class ShooterConstants {
    public static final int FeederMotorCANID = 0;
    public static final double Feeder_GearRatio = 5;
    public static final double Feeder_TargetRPM = 100;
    public static final double FeederPF_P = 0;
    public static final double FeederPF_F = 0;

    public static final int UpperShooterMotorCANID = 35;
    public static final double UpperShooterPF_P = 0;
    public static final double UpperShooterPF_F = 0;
    
    public static final int LowerShooterMotorCANID = 11;
    public static final double LowerShooterPF_P = 0;
    public static final double LowerShooterPF_F = 0;
    
    public static final double Shooter_MaxAccel = 4000;

    public static final double RPM_Tolerance = 50;

  }


  public static class PhysicalProperties 
  {
    public static final double RobotMassKg = 54; // kg
    public static final double RobotMOI = 6; // kg m^2 (Moment of Inertia)
    public static final double wheelCOF = 1.2; // Friction coefficient

  }

  public static class AutoConstants {   
    public static final double PathFollow_maxSpeedMPS = 2; // m/s
    public static final double PathFollow_maxCurrent = 60; // Amps
    public static final PIDConstants PathFollow_Translation_PID = new PIDConstants(5);
    public static final PIDConstants PathFollow_Rotation_PID = new PIDConstants(5);

    public static final double       ApproachPose_Default_maxSpeedMPS = 4;
    public static final Rotation2d   ApproachPose_Default_maxAngVelocity = Rotation2d.fromDegrees(360*2);
    public static final Rotation2d   ApproachPose_Default_maxAngAccel = Rotation2d.fromDegrees(720*1000);

    public static final double       ApproachPose_Default_translationErrorThresholdM = 0.025;
    public static final Rotation2d   ApproachPose_Default_rotationErrorThreshold = Rotation2d.fromDegrees(2);

    public static final PIDConstants ApproachPose_Translation_PID = new PIDConstants(4);
    public static final PIDConstants ApproachPose_Rotation_PID = new PIDConstants(4);


  }

  public static class DriveConstants {
    public static final Pose2d defaultStartPose = new Pose2d(2,1.5, Rotation2d.kZero);

    // TODO -- Shuffleboard selection ?

    public static final double MaxDriveSpeed = 3.5;
    
    // m/s
    public static final double SwerveDesaturationThreshold = 6; // m/s

    // Calculated for each axis, seperately
    public static final double MaxDriveAccel = 5; // m/s^2
    public static final double MaxDriveDeccel = 30; // m/s^2

    public static final double MaxRotSpeed = Math.PI * 2 * 0.5; // rad/s
    public static final double MaxRotAccel = Math.PI * 2 * 6; // rad/s^2
    public static final double MaxRotDeccel = Math.PI * 2 * 6; // rad/s^2
    

    public static final double JOYDeadzone_X = 0.05;
    public static final double JOYDeadzone_Y = 0.05;
    public static final double JOYDeadzone_Rot = 0.05;
    public static final boolean SquareInputs = true;
  }

  public static class SwerveConstants {

    public enum DriveGearRatioOption {
      R1, R2, R3
    }

    public static final double TrackWidthM = 0.63;
    public static final double GearRatio_Angle = 287.0 / 11.0;
    public static final Map<DriveGearRatioOption, Double> GearRatioMap_Drive = Map.of(
        DriveGearRatioOption.R1, 7.03,
        DriveGearRatioOption.R2, 6.03,
        DriveGearRatioOption.R3, 5.27);

    public static final double WheelRadiusM = Units.inchesToMeters(2);

    public static final double AnglePIDV_P = 3;
    public static final double AnglePIDV_I = 0;
    public static final double AnglePIDV_D = 0;
    public static final double AnglePIDV_V = 0;
    public static final double AnglePeakVoltage = 9;

    public static final double DrivePIDV_P = 0.0000005; // Old value: 0.05 -- TODO: test new P value
    public static final double DrivePIDV_I = 0;
    public static final double DrivePIDV_D = 0;
    public static final double DrivePIDV_V = 0.12;
    public static final double DrivePeakVoltage = 12;

    public static final Map<SwerveModuleLocation, Integer> AngleCANIDMap = Map.of(
        SwerveModuleLocation.FRONT_LEFT, 12,
        SwerveModuleLocation.FRONT_RIGHT, 14,
        SwerveModuleLocation.BACK_LEFT, 16,
        SwerveModuleLocation.BACK_RIGHT, 18);

    public static final Map<SwerveModuleLocation, Integer> DriveCANIDMap = Map.of(
        SwerveModuleLocation.FRONT_LEFT, 11,
        SwerveModuleLocation.FRONT_RIGHT, 13,
        SwerveModuleLocation.BACK_LEFT, 15,
        SwerveModuleLocation.BACK_RIGHT, 17);

    // Absolute encoder not implemented yet
    public static final int ABSENCPORTID_FL = -1;
    public static final int ABSENCPORTID_FR = -1;
    public static final int ABSENCPORTID_BL = -1;
    public static final int ABSENCPORTID_BR = -1;

    public static final double ABSENCOFFSET_FL = 0;
    public static final double ABSENCOFFSET_FR = 0;
    public static final double ABSENCOFFSET_BL = 0;
    public static final double ABSENCOFFSET_BR = 0;

  }
}
