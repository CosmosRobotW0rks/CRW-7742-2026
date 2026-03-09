package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.EntryUtils;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.Logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.auto.AutoHelper;
import frc.robot.subsystems.drivetrain.swerve.SwerveSubsystem;

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
  private boolean calibrating = false;

  private final SwerveSubsystem swerveSubsystem;

  private boolean feederState = false;
  private double feederUnlockAfter = 0;
  private double lastFeederStart = 0;


  public ShooterSubsystem(SwerveSubsystem swerveSubsystem) {

    this.swerveSubsystem = swerveSubsystem;

    // Load Calibration
    calibration = new ShooterCalibration();
    int selectedSlot = ShooterCalibration.getSelectedCalibrationSlot();
    if(selectedSlot == -1)
    {
      Logging.stickyError("Shooter Calibration Error", "An error occured while reading the selected shootercalib slot");
    } else {
      ShooterCalibration loadedCalibration = ShooterCalibration.loadFromSlot(selectedSlot);
      if(loadedCalibration == null && selectedSlot != 0)
      {
        Logging.stickyError("Shooter Calibration Error", "An error occured while reading shootercalib slot " + selectedSlot);
      }
      else if(loadedCalibration != null) {
        calibration = loadedCalibration;
        Logging.infoMsg("Shooter Calibration Loaded", "Successfully loaded shootercalib slot " + selectedSlot);
      }
      else
      {
        Logging.stickyWarning("Shooter Calibration Warning", "Shooter calibration not found. Shooter won't be able to shoot.");
      }
    }

    // Motors
    upperMotor = new SparkMax(Constants.ShooterConstants.UpperShooterMotorCANID, MotorType.kBrushless); // MAXMotion
                                                                                                        // Velocity
    SparkMaxConfig upperMotorConfig = new SparkMaxConfig();
    upperMotorConfig.closedLoop.feedForward.kV(Constants.ShooterConstants.UpperShooterPF_F);
    upperMotorConfig.closedLoop.p(Constants.ShooterConstants.UpperShooterPF_P);
    upperMotor.configure(upperMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    lowerMotor = new SparkMax(Constants.ShooterConstants.LowerShooterMotorCANID, MotorType.kBrushless); // MAXMotion
                                                                                                        // Velocity
    SparkMaxConfig lowerMotorConfig = new SparkMaxConfig();
    lowerMotorConfig.closedLoop.feedForward.kV(Constants.ShooterConstants.LowerShooterPF_F);
    lowerMotorConfig.closedLoop.p(Constants.ShooterConstants.LowerShooterPF_P);
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
          upperMotor.stopMotor();
          lowerMotor.stopMotor();
          upperMotorSimRPM = 0;
          lowerMotorSimRPM = 0;
        });
  }

  public Command FeedCommand() {
    return Commands.runEnd(
        () -> toggleFeeder(true),
        () -> toggleFeeder(false));
  }


  // CALIB SUBSYS CONNECTION

  public ShooterCalibration getCalibration() {
    return calibration;
  }

  public void reloadCalibration() {
    int selectedSlot = ShooterCalibration.getSelectedCalibrationSlot();
    if (selectedSlot == -1) {
      Logging.stickyError("Shooter Calibration Error", "An error occured while reading the selected shootercalib slot during reload");
      return;
    }

    ShooterCalibration loadedCalibration = ShooterCalibration.loadFromSlot(selectedSlot);
    if (loadedCalibration == null) {
      calibration = new ShooterCalibration();
      Logging.stickyWarning("Shooter Calibration Warning", "Failed to reload shooter calibration slot " + selectedSlot + ". Using empty calibration.");
    } else {
      calibration = loadedCalibration;
      Logging.infoMsg("Shooter Calibration Reloaded", "Reloaded shootercalib slot " + selectedSlot);
    }

    refreshShooterTargetRPMs();
  }

  public void setCalibrationRPMs(double upperRPM, double lowerRPM)
  {
    calibrating = true;

    upperMotorTargetRPM = upperRPM;
    lowerMotorTargetRPM = lowerRPM;
  }

  public void exitCalibrationMode()
  {
    calibrating = false;
    refreshShooterTargetRPMs();
  }


  // UTILS
  private void refreshShooterTargetRPMs()
  {
    if(calibrating) return;

    double shooterDistance = getShooterDistanceToHub();

    if(calibration == null) return;

    Translation2d rpms = calibration.getRPMForDistance(shooterDistance);

    if(rpms == null) return;

    upperMotorTargetRPM = rpms.getX();
    lowerMotorTargetRPM = rpms.getY();
  }

  public double getRobotDistanceToHub()
  {
    double dist = -1;

    Pose2d robotPose = swerveSubsystem.getRobotPose();

    if (robotPose != null) {
      Translation2d robotPosition = robotPose.getTranslation();
      Translation2d targetPosition = FieldUtils.GetAllianceBasedHubCenter();
      
      dist = robotPosition.getDistance(targetPosition);
    }

    return dist;

  }


  public double getShooterDistanceToHub()
  {
    ShooterDistCalcMode mode = distCalcModeChooser.getSelected();

    double dist = -1;

    if(mode == ShooterDistCalcMode.MANUAL) {
      dist = manualShooterDistanceEntry.get();
    }
    else
    {
      Pose2d robotPose = swerveSubsystem.getRobotPose();
      dist = AutoHelper.GetShooterDistanceToHub(robotPose);
    }

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
  public double getUpperMotorRPM() {
    return Robot.isSimulation() ? upperMotorSimRPM : upperMotor.getEncoder().getVelocity();
  }

  public double getLowerMotorRPM() {
    return Robot.isSimulation() ? lowerMotorSimRPM : lowerMotor.getEncoder().getVelocity();
  }

  private double getFeederMotorRPM() {
    return Robot.isSimulation() ? feederMotorSimRPM : feederMotor.getEncoder().getVelocity();
  }

  // SET RPMS
  private void setUpperMotorRPM(double rpm) {
    upperMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
  }

  private void setLowerMotorRPM(double rpm) {
    lowerMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
  }

  private void toggleFeeder(boolean state) {
    if(state) lastFeederStart = Timer.getFPGATimestamp();
    feederState = state;
    //feederMotor.getClosedLoopController().setSetpoint(state ? Constants.ShooterConstants.Feeder_TargetVoltage : 0, ControlType.kVoltage);


  }

  // PERIODIC

  boolean onePeriodicCheck = false;

  @Override
  public void periodic() {

    double now = Timer.getFPGATimestamp();

    if(feederMotor.getOutputCurrent() > 70)
    {
      if(onePeriodicCheck)
      {
        feederMotor.getClosedLoopController().setSetpoint(0, ControlType.kVelocity);
        feederUnlockAfter = now + 0.15;
        onePeriodicCheck = false;
      }
      else onePeriodicCheck = true;
    }
    else onePeriodicCheck = false;

    if(feederState && feederUnlockAfter < now)
      feederMotor.getClosedLoopController().setSetpoint(Constants.ShooterConstants.Feeder_TargetRPM, ControlType.kVelocity);
    else 
      feederMotor.getClosedLoopController().setSetpoint(0, ControlType.kVelocity);


    double robotDistanceToHub = getRobotDistanceToHub();
    double shooterDistanceToHub = getShooterDistanceToHub();
    refreshShooterTargetRPMs();

    SmartDashboard.putBoolean("Shooter/ShooterAtTarget", isShooterAtTargetRPM());
    SmartDashboard.putBoolean("Shooter/FeederAtTarget", isFeederAtTargetRPM());
    
    SmartDashboard.putNumber("Shooter/RPMs/UpperShooterRPM", getUpperMotorRPM());
    SmartDashboard.putNumber("Shooter/RPMs/LowerShooterRPM", getLowerMotorRPM());
    SmartDashboard.putNumber("Shooter/RPMs/FeederRPM", getFeederMotorRPM());

    SmartDashboard.putNumber("Shooter/TargetRPMs/UpperShooterTargetRPM", upperMotorTargetRPM);
    SmartDashboard.putNumber("Shooter/TargetRPMs/LowerShooterTargetRPM", lowerMotorTargetRPM);

    SmartDashboard.putNumber("Shooter/CalibrationSlot", calibration == null ? -1 : calibration.getSlot());

    // NOTE: Only shooter distance to hub is influenced by the manual distance. Robot distance to hub is always calculated with the drivetrain pose.
    SmartDashboard.putNumber("Shooter/ShooterDistanceToHub", shooterDistanceToHub); 
    SmartDashboard.putNumber("Shooter/RobotDistanceToHub", robotDistanceToHub);
  }

  @Override
  public void simulationPeriodic() {
    if(Math.abs(getUpperMotorRPM() - upperMotorTargetRPM) > Constants.ShooterConstants.RPM_Tolerance)
    upperMotorSimRPM += Math.signum(upperMotor.getClosedLoopController().getSetpoint()-upperMotorSimRPM) * 0.02 * 4000;

    if(Math.abs(getLowerMotorRPM() - lowerMotorTargetRPM) > Constants.ShooterConstants.RPM_Tolerance)
    lowerMotorSimRPM += Math.signum(lowerMotor.getClosedLoopController().getSetpoint()-lowerMotorSimRPM) * 0.02 * 4000;

    feederMotorSimRPM = feederMotor.getClosedLoopController().getSetpoint();
  }

}