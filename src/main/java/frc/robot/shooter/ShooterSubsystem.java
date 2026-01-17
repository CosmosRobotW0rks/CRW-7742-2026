package frc.robot.shooter;

import java.security.PublicKey;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.EntryUtils;
import frc.robot.utils.Logging;

public class ShooterSubsystem extends SubsystemBase {

  private enum ShooterDistCalcMode{
    MANUAL,
    VISION_BASED
  };

  private final SendableChooser<ShooterDistCalcMode> distCalcModeChooser = new SendableChooser<ShooterDistCalcMode>();
  private final DoubleEntry manualShooterDistanceEntry = EntryUtils.createDoubleEntry("Shooter/ManualDistance", 2);

  private SparkMax upperMotor;
  private SparkMax lowerMotor;
  private SparkMax feederMotor;
  
  double upperMotorSimRPM = 0;
  double lowerMotorSimRPM = 0;
  double feederMotorSimRPM = 0;

  private ShooterCalibration calibration;

  private double upperMotorTargetRPM = 0;
  private double lowerMotorTargetRPM = 0;

  private double distanceToHub = 0;



  public ShooterSubsystem() {


    // Load Calibration
    int selectedSlot = ShooterCalibration.getSelectedCalibrationSlot();
    if(selectedSlot == -1)
    {
      Logging.stickyError("Shooter Calibration Error", "An error occured while reading the selected shootercalib slot");
      return;
    }

    calibration = ShooterCalibration.loadFromSlot(selectedSlot);
    if(calibration == null && selectedSlot != 0)
    {
      Logging.stickyError("Shooter Calibration Error", "An error occured while reading shootercalib slot " + selectedSlot);
      return;
    }

    if(calibration != null) Logging.infoMsg("Shooter Calibration Loaded", "Successfully loaded shootercalib slot " + selectedSlot);
    else
    {
      Logging.stickyWarning("Shooter Calibration Warning", "Shooter calibration not found. Shooter won't be able to shoot.");
    }

    // Motors
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


    refreshShooterTargetRPMs();

    distCalcModeChooser.setDefaultOption("Vision", ShooterDistCalcMode.VISION_BASED);
    distCalcModeChooser.addOption("Manual", ShooterDistCalcMode.MANUAL);

    SmartDashboard.putData("Shooter/ShooterDistCalcMode", distCalcModeChooser);
  }

  // TRIGGERS
  public Trigger isReadyToShoot() {
    return new Trigger(() -> isShooterAtTargetRPM());
  }

  // COMMANDS

  public Command prepareShooterCommand() {
    return Commands.runEnd(
        () -> {
          refreshShooterTargetRPMs();
          setUpperMotorRPM(upperMotorTargetRPM);
          setLowerMotorRPM(lowerMotorTargetRPM);
        },
        () -> {
          setUpperMotorRPM(0);
          setLowerMotorRPM(0);
        });
  }

  public Command FeedCommand() {
    return Commands.runEnd(
        () -> toggleFeeder(true),
        () -> toggleFeeder(false));
  }

  // UTILS
  public void refreshShooterTargetRPMs()
  {
    double distance = getDistanceToHub();

    if(calibration == null) return;
    
    double rpm = calibration.getRPMForDistance(distance);

    upperMotorTargetRPM = rpm;
    lowerMotorTargetRPM = rpm;
  }


  // HELPERS
  public double getDistanceToHub()
  {
    ShooterDistCalcMode mode = distCalcModeChooser.getSelected();
    
    double dist = -1;

    if(mode == ShooterDistCalcMode.MANUAL) dist = manualShooterDistanceEntry.get();
    else // VISION_BASED
    {
      // TODO: Get from vision
    }

    distanceToHub = dist;
    return dist;

  }

  // BOOL CHECKS
  private boolean isFeederAtTargetRPM() {
    return Math.abs(getFeederMotorRPM() - Constants.ShooterConstants.Feeder_TargetRPM) < Constants.ShooterConstants.RPM_Tolerance;
  }

  private boolean isShooterAtTargetRPM() {
    return Math.abs(getUpperMotorRPM() - upperMotorTargetRPM) < Constants.ShooterConstants.RPM_Tolerance && Math.abs(getLowerMotorRPM() - lowerMotorTargetRPM) < Constants.ShooterConstants.RPM_Tolerance;
  }

  // GET RPMS
  private double getUpperMotorRPM() {
    return Robot.isSimulation() ? upperMotorSimRPM : upperMotor.getEncoder().getVelocity();
  }

  private double getLowerMotorRPM() {
    return Robot.isSimulation() ? lowerMotorSimRPM : lowerMotor.getEncoder().getVelocity();
  }

  private double getFeederMotorRPM() {
    return Robot.isSimulation() ? feederMotorSimRPM : feederMotor.getEncoder().getVelocity();
  }

  // SET RPMS
  private void setUpperMotorRPM(double rpm) {

    upperMotor.getClosedLoopController().setSetpoint(rpm, Robot.isSimulation() ? ControlType.kVelocity : ControlType.kMAXMotionVelocityControl);
  }

  private void setLowerMotorRPM(double rpm) {
    lowerMotor.getClosedLoopController().setSetpoint(rpm, Robot.isSimulation() ? ControlType.kVelocity : ControlType.kMAXMotionVelocityControl);
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

    SmartDashboard.putNumber("Shooter/CalibrationSlot", calibration == null ? -1 : calibration.getSlot());

    SmartDashboard.putNumber("Shooter/DistanceToHub", distanceToHub);
  }

  @Override
  public void simulationPeriodic() {
    upperMotorSimRPM += Math.signum(upperMotor.getClosedLoopController().getSetpoint()-upperMotorSimRPM) * 0.02 * Constants.ShooterConstants.Shooter_MaxAccel;
    lowerMotorSimRPM += Math.signum(lowerMotor.getClosedLoopController().getSetpoint()-lowerMotorSimRPM) * 0.02 * Constants.ShooterConstants.Shooter_MaxAccel;
    feederMotorSimRPM = feederMotor.getClosedLoopController().getSetpoint();
  }

}