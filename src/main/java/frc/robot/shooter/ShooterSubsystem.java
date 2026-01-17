package frc.robot.shooter;

import java.security.PublicKey;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.utils.Elastic;

public class ShooterSubsystem extends SubsystemBase {

  private SparkMax upperMotor;
  private SparkMax lowerMotor;
  private SparkMax feederMotor;

  private ShooterCalibration calibration;

  private double upperMotorTargetRPM = 0;
  private double lowerMotorTargetRPM = 0;


  public ShooterSubsystem() {

    // Load Calibration
    int selectedSlot = ShooterCalibration.getSelectedCalibrationSlot();
    if(selectedSlot == -1)
    {
      Elastic.errorMsg("Shooter Calibration Error", "An error occured while reading the selected shootercalib slot");
      return;
    }

    calibration = ShooterCalibration.loadFromSlot(selectedSlot);
    if(calibration == null)
    {
      Elastic.errorMsg("Shooter Calibration Error", "An error occured while reading shootercalib slot " + selectedSlot);
      return;
    }

    upperMotor = new SparkMax(Constants.ShooterConstants.UpperShooterMotorCANID, MotorType.kBrushless); // MAXMotion
                                                                                                        // Velocity
    SparkMaxConfig upperMotorConfig = new SparkMaxConfig();
    upperMotorConfig.closedLoop.feedForward.kV(Constants.ShooterConstants.UpperShooterPF_F);
    upperMotorConfig.closedLoop.p(Constants.ShooterConstants.UpperShooterPF_P);
    upperMotorConfig.closedLoop.maxMotion.maxAcceleration(Constants.ShooterConstants.Shooter_MaxAccel);
    upperMotor.configure(upperMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    lowerMotor = new SparkMax(Constants.ShooterConstants.LowerShooterMotorCANID, MotorType.kBrushless); // MAXMotion
                                                                                                        // Velocity
    SparkMaxConfig lowerMotorConfig = new SparkMaxConfig();
    lowerMotorConfig.closedLoop.feedForward.kV(Constants.ShooterConstants.LowerShooterPF_F);
    lowerMotorConfig.closedLoop.p(Constants.ShooterConstants.LowerShooterPF_P);
    lowerMotorConfig.closedLoop.maxMotion.maxAcceleration(Constants.ShooterConstants.Shooter_MaxAccel);
    lowerMotor.configure(lowerMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    feederMotor = new SparkMax(Constants.ShooterConstants.FeederMotorCANID, MotorType.kBrushless); // Velocity
    SparkMaxConfig feederMotorConfig = new SparkMaxConfig();
    feederMotorConfig.idleMode(IdleMode.kBrake);
    feederMotorConfig.closedLoop.feedForward.kV(Constants.ShooterConstants.FeederPF_F);
    feederMotorConfig.closedLoop.p(Constants.ShooterConstants.FeederPF_P);
    feederMotor.configure(feederMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  // TRIGGERS
  public Trigger readyToShoot() {
    return new Trigger(() -> isShooterAtTargetRPM());
  }

  // COMMANDS

  public Command prepareShooter() {
    return Commands.runEnd(
        () -> {
          refreshShooterTargetRPMs();
          setUpperMotorRPM(upperMotorTargetRPM);
          setLowerMotorRPM(upperMotorTargetRPM);
        },
        () -> {
          setUpperMotorRPM(0);
          setLowerMotorRPM(0);
        },
        this);
  }

  public Command FeedCommand() {
    return Commands.runEnd(
        () -> toggleFeeder(true),
        () -> toggleFeeder(false),
        this);
  }

  // UTILS
  public void refreshShooterTargetRPMs()
  {
    upperMotorTargetRPM = 2000;
    lowerMotorTargetRPM = 2000;
  }

  // BOOL CHECKS
  private boolean isFeederAtTargetRPM() {
    return getFeederMotorRPM() == Constants.ShooterConstants.Feeder_TargetRPM;
  }

  private boolean isShooterAtTargetRPM() {
    return getUpperMotorRPM() == upperMotorTargetRPM && getLowerMotorRPM() == lowerMotorTargetRPM;
  }

  // GET RPMS
  private double getUpperMotorRPM() {
    return upperMotor.getEncoder().getVelocity();
  }

  private double getLowerMotorRPM() {
    return lowerMotor.getEncoder().getVelocity();
  }

  private double getFeederMotorRPM() {
    return feederMotor.getEncoder().getVelocity();
  }

  // SET RPMS
  private void setUpperMotorRPM(double rpm) {
    upperMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
  }

  private void setLowerMotorRPM(double rpm) {
    lowerMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
  }

  private void toggleFeeder(boolean state) {
    feederMotor.getClosedLoopController().setSetpoint(state ? Constants.ShooterConstants.Feeder_TargetRPM : 0, ControlType.kVelocity);
  }

  // PERIODIC

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Shooter/ShooterAtTarget", isShooterAtTargetRPM());
    SmartDashboard.putBoolean("Shooter/FeederAtTarget", isFeederAtTargetRPM());
    
    SmartDashboard.putNumber("Shooter/RPMs/UpperShooterRPM", getUpperMotorRPM());
    SmartDashboard.putNumber("Shooter/RPMs/LowerShooterRPM", getLowerMotorRPM());
    SmartDashboard.putNumber("Shooter/RPMs/FeederRPM", getFeederMotorRPM());

    SmartDashboard.putNumber("Shooter/TargetRPMs/UpperShooterTargetRPM", upperMotorTargetRPM);
    SmartDashboard.putNumber("Shooter/TargetRPMs/LowerShooterTargetRPM", lowerMotorTargetRPM);
  }

  @Override
  public void simulationPeriodic() {
      
  }

}