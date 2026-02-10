package frc.robot.shooter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Logging;

public class ShooterCalibratorSubsystem extends SubsystemBase {

    private ShooterCalibration calibration = new ShooterCalibration();

    private ShooterSubsystem shooterSubsystem;
    
    private boolean calibrating = false;
    private double shooterRPM = 0;

    public ShooterCalibratorSubsystem(ShooterSubsystem subsystem) {
        this.shooterSubsystem = subsystem;


        SmartDashboard.putData("ShooterCalibration/Actions/StartCalibration",StartCalibrationCommand());
        SmartDashboard.putData("ShooterCalibration/Actions/EndCalibration",EndCalibrationCommand());
        SmartDashboard.putData("ShooterCalibration/Actions/SaveCalibration",SaveCalibrationCommand());
        SmartDashboard.putData("ShooterCalibration/Actions/AddMeasurement",AddMeasurementCommand());
    }

    public Command StartCalibrationCommand() {
        return Commands.runOnce(() -> {

            shooterRPM = 0;
            shooterSubsystem.setCalibrationRPM(shooterRPM);

            calibration = new ShooterCalibration();

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
            calibration.save(0, "Default Calibration");
            Logging.infoMsg("Shooter Calibration", "Saved shooter calibration to slot 0");
        }, this);
    }

    public Command IncreaseRPMCommand(double step)
    {
        return Commands.runOnce(() -> {
            shooterRPM += step;
            shooterSubsystem.setCalibrationRPM(shooterRPM);
        }, this);
    }

    public Command AddMeasurementCommand()
    {
        return Commands.runOnce(() -> {
            double distance = shooterSubsystem.getDistanceToHub();
            calibration.addMeasurement(distance, shooterRPM);
            resetGraph();
            Logging.infoMsg("Shooter Calibration", String.format(Locale.US, "Added Measurement\nDist: %.5fm\nRPM: %.1f", distance, shooterRPM));
        }, this);
    }




    private void resetGraph() {
        List<Translation2d> knownPoints = new ArrayList<>();
        List<Translation2d> predictedPoints = new ArrayList<>();

        Map<Double, Double> measurements = calibration.getMeasurements();
        Double[] distances = measurements.keySet().toArray(new Double[0]);
        Double[] rpms = measurements.values().toArray(new Double[0]);

        if (distances.length > 0) {

            for (int i = 0; i < measurements.size(); i++) {
                knownPoints.add(new Translation2d(distances[i], rpms[i]));
            }

            double minDist = Collections.min(Arrays.asList(distances));
            double maxDist = Collections.max(Arrays.asList(distances));

            for (double d = minDist; d <= maxDist; d += 0.01) {
                double rpm = calibration.getRPMForDistance(d);
                predictedPoints.add(new Translation2d(d, rpm));
            }

        }

        SmartDashboard.putString("ShooterCalibration/KnownPoints", buildPointArrString(knownPoints.toArray(new Translation2d[0])));
        SmartDashboard.putString("ShooterCalibration/PredictedPoints", buildPointArrString(predictedPoints.toArray(new Translation2d[0])));
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
        SmartDashboard.putNumber("ShooterCalibration/RPM", shooterRPM);
        SmartDashboard.putNumber("ShooterCalibrator/TotalMeasurements", calibration.getMeasurementCount());
    }
}
