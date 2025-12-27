package frc.robot.drivetrain;

import java.util.Map;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivetrain.Common.SwerveModuleConfiguration;
import frc.robot.drivetrain.Common.SwerveModuleLocation;
import frc.robot.drivetrain.Common.SwerveMotorConfig;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.DriveGearRatioOption;

public class SwerveDrivetrain extends SubsystemBase {

    Map<SwerveModuleLocation, SwerveModule> modules;

    private DriveGearRatioOption driveGearRatioOption = DriveGearRatioOption.R2;

    public SwerveDrivetrain() {
        initModules();
    }

    private void initModules() {
        initModule(SwerveModuleLocation.FRONT_LEFT);
        initModule(SwerveModuleLocation.FRONT_RIGHT);
        initModule(SwerveModuleLocation.BACK_LEFT);
        initModule(SwerveModuleLocation.BACK_RIGHT);
    }

    private void initModule(SwerveModuleLocation location)
    {
        SwerveAngleKraken angleMotor = new SwerveAngleKraken();
        SwerveDriveKraken driveMotor = new SwerveDriveKraken();

        SwerveModule module = new SwerveModule();

        angleMotor.init(new SwerveMotorConfig(
         SwerveConstants.AngleCANIDMap.get(location),

         SwerveConstants.GearRatio_Angle,
         SwerveConstants.WheelRadiusM,
         
         SwerveConstants.AnglePeakVoltage,
         
         SwerveConstants.AnglePIDV_P,
         SwerveConstants.AnglePIDV_I,
         SwerveConstants.AnglePIDV_D,
         SwerveConstants.AnglePIDV_V
         ));

        driveMotor.init(new SwerveMotorConfig(
          SwerveConstants.DriveCANIDMap.get(location),
 
          SwerveConstants.GearRatioMap_Drive.get(driveGearRatioOption),
          SwerveConstants.WheelRadiusM,
          
          SwerveConstants.DrivePeakVoltage,
          
          SwerveConstants.DrivePIDV_P,
          SwerveConstants.DrivePIDV_I,
          SwerveConstants.DrivePIDV_D,
          SwerveConstants.DrivePIDV_V
          ));

          module.init(new SwerveModuleConfiguration(location, angleMotor, driveMotor));

    }
}
