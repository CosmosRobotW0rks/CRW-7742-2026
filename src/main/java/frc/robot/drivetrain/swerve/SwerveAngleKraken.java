package frc.robot.drivetrain.swerve;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.drivetrain.swerve.common.BaseSwerveAngleMotor;
import frc.robot.drivetrain.swerve.common.SwerveMotorConfig;

public class SwerveAngleKraken implements BaseSwerveAngleMotor{

    SwerveMotorConfig motorConfig;

    PositionVoltage request = new PositionVoltage(0).withSlot(0);

    DCMotorSim motorSimModel;
    TalonFX talonFX;

    double targetMotorRot = 0;

    @Override
    public boolean init(SwerveMotorConfig config)
    {
        motorConfig = config;

        talonFX = new TalonFX(config.canID());

        motorSimModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, motorConfig.gearRatio()), DCMotor.getKrakenX60Foc(1));

        var talonCfg = new TalonFXConfiguration();
        talonCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;


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


    // SIMULATION

    @Override
    public void simulationPeriodic() {
        var talonFXSim = talonFX.getSimState();
        
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        motorSimModel.update(0.02);

        talonFXSim.setRawRotorPosition(motorSimModel.getAngularPosition().times(motorConfig.gearRatio()));
        talonFXSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(motorConfig.gearRatio()));
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
