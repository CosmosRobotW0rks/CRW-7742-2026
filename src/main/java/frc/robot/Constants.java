// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import frc.robot.drivetrain.Common.SwerveModuleLocation;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveConstants {

    public enum DriveGearRatioOption {
      R1, R2, R3
    }

    public static final double GearRatio_Angle = 287 / 11;
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

    public static final double DrivePIDV_P = 0.1;
    public static final double DrivePIDV_I = 0;
    public static final double DrivePIDV_D = 0;
    public static final double DrivePIDV_V = 0.12;
    public static final double DrivePeakVoltage = 7;

    public static final Map<SwerveModuleLocation, Integer> AngleCANIDMap = Map.of(
        SwerveModuleLocation.FRONT_LEFT, -1,
        SwerveModuleLocation.FRONT_RIGHT, -1,
        SwerveModuleLocation.BACK_LEFT, -1,
        SwerveModuleLocation.BACK_RIGHT, -1);

    public static final Map<SwerveModuleLocation, Integer> DriveCANIDMap = Map.of(
        SwerveModuleLocation.FRONT_LEFT, -1,
        SwerveModuleLocation.FRONT_RIGHT, -1,
        SwerveModuleLocation.BACK_LEFT, -1,
        SwerveModuleLocation.BACK_RIGHT, -1);

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
