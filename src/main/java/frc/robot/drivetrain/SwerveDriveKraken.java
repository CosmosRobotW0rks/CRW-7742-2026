package frc.robot.drivetrain;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.drivetrain.Common.BaseSwerveDriveMotor;
import frc.robot.drivetrain.Common.SwerveMotorConfig;

public class SwerveDriveKraken implements BaseSwerveDriveMotor {

    SwerveMotorConfig motorConfig;

    VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

    TalonFX talonFX;

    double gearRatio;

    double targetMotorRPS = 0;

    @Override
    public boolean init(SwerveMotorConfig config)
    {
        motorConfig = config;

        talonFX = new TalonFX(config.canID());

        var talonCfg = new TalonFXConfiguration();
        var slot0Configs = new Slot0Configs();

        slot0Configs.kP = config.kP();
        slot0Configs.kI = config.kI();
        slot0Configs.kD = config.kD();
        slot0Configs.kV = config.kV();

        talonCfg.Voltage.PeakForwardVoltage = config.peakVoltage();
        talonCfg.Voltage.PeakReverseVoltage = -config.peakVoltage();

        talonCfg.Slot0 = slot0Configs;

        gearRatio = config.gearRatio();
        
        StatusCode status = talonFX.getConfigurator().apply(talonCfg);
        if(status.isError())
        {
            DriverStation.reportError("Failed to initialize Swerve TalonFX (ID: " + config.canID() + ")", false);
            return false;
        }

        return true;
    }

    @Override
    public void stop() {
        talonFX.stopMotor();
    }


    // GETTERS

    @Override
    public double getTotalDistance() {
        double motorRot = talonFX.getPosition().getValueAsDouble();

        double meters = motorRotToMeters(motorRot);

        return meters;
        
    }

    @Override
    public double getCurrentVelocity() {     
        double motorRPS = talonFX.getVelocity().getValueAsDouble();

        double metersPerSecond = motorRotToMeters(motorRPS);

        return metersPerSecond;
    }

    @Override
    public double getTargetVelocity() {
        double metersPerSecond = motorRotToMeters(targetMotorRPS);

        return metersPerSecond;
    }


    // SETTERS

    @Override
    public void setTargetVelocity(double velocityMetersPerSecond) {
        double motorRPS = metersToMotorRot(velocityMetersPerSecond);

        request.Velocity = motorRPS;

        StatusCode code = talonFX.setControl(request);

        if(code.isError())
        {
            DriverStation.reportError("Failed to set Swerve TalonFX (ID: " + talonFX.getDeviceID() + ") velocity", false);
        }

        targetMotorRPS = motorRPS;
        
    }


    // UTILS
    
    private double motorRotToMeters(double motorRot)
    {
        double meters = (motorRot * 2 * Math.PI * motorConfig.wheelRadius()) / motorConfig.gearRatio();

        return meters;
    }

    private double metersToMotorRot(double meters)
    {
        double motorRot = meters / (2 * Math.PI * motorConfig.wheelRadius() * motorConfig.gearRatio());

        return motorRot;
    }

}
