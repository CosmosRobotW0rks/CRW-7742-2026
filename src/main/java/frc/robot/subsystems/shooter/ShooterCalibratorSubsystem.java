package frc.robot.subsystems.shooter;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.EntryUtils;
import frc.robot.utils.Logging;

public class ShooterCalibratorSubsystem extends SubsystemBase {

    private ShooterCalibration calibration;

    private final ShooterSubsystem shooterSubsystem;

    private final DoubleEntry rpmStepEntry = EntryUtils.createDoubleEntry("ShooterCalibration/Settings/RPMStep", 50.0);
    
    private boolean calibrating = false;
    private double upperRPM = 0;
    private double lowerRPM = 0;

    public ShooterCalibratorSubsystem(ShooterSubsystem subsystem) {
        this.shooterSubsystem = subsystem;
        this.calibration = subsystem.getCalibration();


        SmartDashboard.putData("ShooterCalibration/Actions/StartCalibration",StartCalibrationCommand());
        SmartDashboard.putData("ShooterCalibration/Actions/EndCalibration",EndCalibrationCommand());
        SmartDashboard.putData("ShooterCalibration/Actions/SaveCalibration",SaveCalibrationCommand());
        SmartDashboard.putData("ShooterCalibration/Actions/AddMeasurement",AddMeasurementCommand());
        SmartDashboard.putData("ShooterCalibration/Actions/IncreaseUpperRPM",IncreaseUpperRPMCommand());
        SmartDashboard.putData("ShooterCalibration/Actions/DecreaseUpperRPM",DecreaseUpperRPMCommand());
        SmartDashboard.putData("ShooterCalibration/Actions/IncreaseLowerRPM",IncreaseLowerRPMCommand());
        SmartDashboard.putData("ShooterCalibration/Actions/DecreaseLowerRPM",DecreaseLowerRPMCommand());
        SmartDashboard.putData("ShooterCalibration/Actions/UndoLastMeasurement", UndoLastMeasurementCommand());
        SmartDashboard.putData("ShooterCalibration/Actions/ClearAllMeasurements", ClearAllMeasurementsCommand());
    }

    public Command StartCalibrationCommand() {
        return Commands.runOnce(() -> {

            upperRPM = 0;
            lowerRPM = 0;
            shooterSubsystem.setCalibrationRPMs(upperRPM, lowerRPM);

            resetGraph();


            calibrating = true;
        }, this);
    }

    public Command EndCalibrationCommand() {
        return Commands.runOnce(() -> {
            
            calibrating = false;
            shooterSubsystem.exitCalibrationMode();
        }, this);
    }

    public Command SaveCalibrationCommand()
    {
        return Commands.runOnce(() -> {
            ShooterCalibration sharedCalibration = getCalibration();
            if (sharedCalibration == null) {
                Logging.warningMsg("Shooter Calibration", "No calibration data to save.");
                return;
            }

            boolean saved = sharedCalibration.save(0, "Default Calibration");
            if (saved) {
                shooterSubsystem.reloadCalibration();
                Logging.infoMsg("Shooter Calibration", "Saved shooter calibration to slot 0");
            } else {
                Logging.warningMsg("Shooter Calibration", "Failed to save shooter calibration to slot 0");
            }
        }, this);
    }

    public Command IncreaseUpperRPMCommand()
    {
        return adjustUpperRPMCommand(1);
    }

    public Command DecreaseUpperRPMCommand()
    {
        return adjustUpperRPMCommand(-1);
    }

    public Command IncreaseLowerRPMCommand()
    {
        return adjustLowerRPMCommand(1);
    }

    public Command DecreaseLowerRPMCommand()
    {
        return adjustLowerRPMCommand(-1);
    }

    private Command adjustUpperRPMCommand(double delta)
    {
        return Commands.runOnce(() -> {
            double step = rpmStepEntry.get();
            upperRPM += delta * step;
            shooterSubsystem.setCalibrationRPMs(upperRPM, lowerRPM);
        }, this);
    }

    private Command adjustLowerRPMCommand(double delta)
    {
        return Commands.runOnce(() -> {
            double step = rpmStepEntry.get();
            lowerRPM += delta * step;
            shooterSubsystem.setCalibrationRPMs(upperRPM, lowerRPM);
        }, this);
    }

    public Command AddMeasurementCommand() {
        return Commands.runOnce(() -> {

            ShooterCalibration sharedCalibration = getCalibration();
            if (sharedCalibration == null) {
                Logging.warningMsg("Shooter Calibration", "Calibration data not available, cannot add measurement");
                return;
            }

            double shooterDistance = shooterSubsystem.getShooterDistanceToHub();

            double currentUpperRPM = this.upperRPM; 
            double currentLowerRPM = this.lowerRPM;
            sharedCalibration.addMeasurement(shooterDistance, currentUpperRPM, currentLowerRPM);
            
            resetGraph();
            Logging.infoMsg("Shooter Calibration", String.format(Locale.US, 
                "Ölçüm Eklendi - Mesafe: %.2fm, Üst RPM: %.0f, Alt RPM: %.0f", 
                shooterDistance, currentUpperRPM, currentLowerRPM));
        }, this);
        
    }

    public Command UndoLastMeasurementCommand()
    {
        return Commands.runOnce(() -> {
            ShooterCalibration sharedCalibration = getCalibration();
            if (sharedCalibration == null) {
                Logging.warningMsg("Shooter Calibration", "No calibration data to undo.");
                return;
            }

            boolean removed = sharedCalibration.removeLastMeasurement();
            resetGraph();
            if (removed) {
                Logging.infoMsg("Shooter Calibration", "Removed last calibration measurement");
            } else {
                Logging.warningMsg("Shooter Calibration", "No calibration measurements to remove");
            }
        }, this);
    }

    public Command ClearAllMeasurementsCommand()
    {
        return Commands.runOnce(() -> {
            ShooterCalibration sharedCalibration = getCalibration();
            if (sharedCalibration == null) {
                Logging.warningMsg("Shooter Calibration", "No calibration data to clear.");
                return;
            }

            sharedCalibration.clearAllMeasurements();
            String description = sharedCalibration.getDescription();
            if (description == null) {
                description = "";
            }
            sharedCalibration.save(0, description);
            resetGraph();
            Logging.infoMsg("Shooter Calibration", "Cleared all calibration measurements");
        }, this);
    }



    private void resetGraph() {
        List<Translation2d> knownUpperPoints = new ArrayList<>();
        List<Translation2d> knownLowerPoints = new ArrayList<>();
        List<Translation2d> predictedUpperPoints = new ArrayList<>();
        List<Translation2d> predictedLowerPoints = new ArrayList<>();

        ShooterCalibration sharedCalibration = getCalibration();
        if (sharedCalibration != null) {
            Map<Double, Translation2d> measurements = sharedCalibration.getMeasurements();
            List<Double> sortedShooterDistances = new ArrayList<>(measurements.keySet());
            Collections.sort(sortedShooterDistances);

            if (!sortedShooterDistances.isEmpty()) {

                for (Double shooterDistance : sortedShooterDistances) {
                    Translation2d rpms = measurements.get(shooterDistance);
                    if (rpms == null) continue;
                    knownUpperPoints.add(new Translation2d(shooterDistance, rpms.getX()));
                    knownLowerPoints.add(new Translation2d(shooterDistance, rpms.getY()));
                }

                double minDist = sortedShooterDistances.get(0);
                double maxDist = sortedShooterDistances.get(sortedShooterDistances.size() - 1);

                for (double shooterDistance = minDist; shooterDistance <= maxDist; shooterDistance += 0.01) {
                    Translation2d rpmPair = sharedCalibration.getRPMForDistance(shooterDistance);
                    if(rpmPair == null) continue;
                    predictedUpperPoints.add(new Translation2d(shooterDistance, rpmPair.getX()));
                    predictedLowerPoints.add(new Translation2d(shooterDistance, rpmPair.getY()));
                }
            }
        }

        SmartDashboard.putString("ShooterCalibration/UpperKnownPoints", buildPointArrString(knownUpperPoints.toArray(new Translation2d[0])));
        SmartDashboard.putString("ShooterCalibration/UpperPredictedPoints", buildPointArrString(predictedUpperPoints.toArray(new Translation2d[0])));
        SmartDashboard.putString("ShooterCalibration/LowerKnownPoints", buildPointArrString(knownLowerPoints.toArray(new Translation2d[0])));
        SmartDashboard.putString("ShooterCalibration/LowerPredictedPoints", buildPointArrString(predictedLowerPoints.toArray(new Translation2d[0])));
    }

    private ShooterCalibration getCalibration() {
        calibration = shooterSubsystem.getCalibration();
        return calibration;
    }

    private String buildPointArrString(Translation2d[] points)
    {
        StringBuilder sb = new StringBuilder();
        sb.append("[");
        for(int i = 0; i < points.length; i++)
        {
            sb.append(String.format(Locale.US, "(%.5f, %.5f)", points[i].getX(), points[i].getY()));
            if(i < points.length - 1) sb.append(",");
        }
        sb.append("]");
        return sb.toString();
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("ShooterCalibration/Calibrating", calibrating);
        SmartDashboard.putNumber("ShooterCalibration/UpperRPM", upperRPM);
        SmartDashboard.putNumber("ShooterCalibration/LowerRPM", lowerRPM);
        ShooterCalibration sharedCalibration = getCalibration();
        int measurementCount = sharedCalibration == null ? 0 : sharedCalibration.getMeasurementCount();
        SmartDashboard.putNumber("ShooterCalibrator/TotalMeasurements", measurementCount);
    }
}