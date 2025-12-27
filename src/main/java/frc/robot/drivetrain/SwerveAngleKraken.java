package frc.robot.drivetrain;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.drivetrain.Common.BaseSwerveAngleMotor;
import frc.robot.drivetrain.Common.SwerveMotorConfig;

public class SwerveAngleKraken implements BaseSwerveAngleMotor{

    SwerveMotorConfig motorConfig;

    PositionVoltage request = new PositionVoltage(0).withSlot(0);

    TalonFX talonFX;

    double targetMotorRot = 0;

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
    public Rotation2d getCurrentAngle()
    {
        double motorRot = talonFX.getPosition().getValueAsDouble();

        return motorRotToSwerveAngle(motorRot);
    }

    @Override
    public Rotation2d getTargetAngle()
    {
        return motorRotToSwerveAngle(targetMotorRot);
    }

    // SETTERS

    @Override
    public void setCurrentAngle(Rotation2d angle)
    {
        double motorRot = swerveAngleToMotorRot(angle);

        StatusCode code = talonFX.getConfigurator().setPosition(motorRot);

        if(code.isError())
        {
            DriverStation.reportError("Failed to set Swerve TalonFX (ID: " + talonFX.getDeviceID() + ") position", false);
        }
    }

    @Override
    public void setTargetAngle(Rotation2d targetAngle)
    {
        targetMotorRot = swerveAngleToMotorRot(targetAngle);
        request = request.withPosition(targetMotorRot);

        StatusCode code = talonFX.setControl(request);

        if(code.isError())
        {
            DriverStation.reportError("Failed to set Swerve TalonFX (ID: " + talonFX.getDeviceID() + ") target angle", false);
        }
    }


    // UTILS


    private Rotation2d motorRotToSwerveAngle(double motorRot)
    {
        double swerveRot = (motorRot / motorConfig.gearRatio());

        return Rotation2d.fromRotations(swerveRot);
    }

    private double swerveAngleToMotorRot(Rotation2d swerveAngle)
    {
        double swerveRot = swerveAngle.getRotations();

        return swerveRot * motorConfig.gearRatio();
    }


}
