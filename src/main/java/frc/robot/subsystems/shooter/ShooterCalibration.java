package frc.robot.subsystems.shooter;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.utils.InterpolatingTreeMap;

public class ShooterCalibration {

    private static final Interpolator<Translation2d> TRANSLATION_INTERPOLATOR = (start, end, t) -> new Translation2d(
            start.getX() + (end.getX() - start.getX()) * t,
            start.getY() + (end.getY() - start.getY()) * t);

    private InterpolatingTreeMap<Double, Translation2d> interpolatingMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(),
            TRANSLATION_INTERPOLATOR);

    private HashMap<Double, Translation2d> measurements = new HashMap<Double, Translation2d>();
    private List<Double> measurementHistory = new ArrayList<>();
    private String description;
    private int slot;

    private record ShooterConfig(int slot) {
    };

    
    private static class ShooterCalib {
        public String description;
        public List<CalibrationRecord> measurements;
        public Map<String, CalibrationRPMs> knownValues;

        public ShooterCalib() {
        }

        public ShooterCalib(String description, List<CalibrationRecord> measurements) {
            this.description = description;
            this.measurements = measurements;
        }

        public List<CalibrationRecord> getRecords() {
            if (measurements != null) {
                return measurements;
            }

            if (knownValues == null) {
                return Collections.emptyList();
            }

            List<CalibrationRecord> records = new ArrayList<>();
            knownValues.forEach((distanceString, rpms) -> {
                if (distanceString == null || rpms == null) {
                    return;
                }

                try {
                    double shooterDistance = Double.parseDouble(distanceString);
                    records.add(new CalibrationRecord(shooterDistance, rpms.upperRPM(), rpms.lowerRPM()));
                } catch (NumberFormatException ignored) {
                }
            });

            records.sort(Comparator.comparingDouble(record -> record.distance));
            return records;
        }
    };

    private record CalibrationRPMs(double upperRPM, double lowerRPM) {
    }

    private static class CalibrationRecord {
        public double distance;
        public double upperRPM;
        public double lowerRPM;

        public CalibrationRecord() {
        }

        public CalibrationRecord(double distance, double upperRPM, double lowerRPM) {
            this.distance = distance;
            this.upperRPM = upperRPM;
            this.lowerRPM = lowerRPM;
        }
    }

    private static final int DEFAULT_SLOT = 0;

    public ShooterCalibration() {
        
    }

    public Translation2d getRPMForDistance(double shooterDistance) {
        if (measurements.isEmpty()) {
            return null;
        }
        return interpolatingMap.get(shooterDistance);
    }

    public int getMeasurementCount() {
        return measurements.size();
    }
    
    public void addMeasurement(double shooterDistance, double upperRPM, double lowerRPM) {
        Translation2d rpms = new Translation2d(upperRPM, lowerRPM);
        measurements.put(shooterDistance, rpms);
        interpolatingMap.put(shooterDistance, rpms);
        measurementHistory.add(shooterDistance);
    }

    public boolean removeMeasurement(double shooterDistance, double upperRPM, double lowerRPM) {
        Translation2d removedMeasurement = measurements.remove(shooterDistance);
        Translation2d removedInterpolated = interpolatingMap.remove(shooterDistance);
        boolean removed = removedMeasurement != null || removedInterpolated != null;
        if (removed) {
            measurementHistory.remove(shooterDistance);
        }
        return removed;
    }

    public boolean removeLastMeasurement() {
        if (measurementHistory.isEmpty()) {
            return false;
        }

        double lastShooterDistance = measurementHistory.remove(measurementHistory.size() - 1);
        Translation2d rpms = measurements.remove(lastShooterDistance);

        if (rpms == null) {
            return false;
        }

        interpolatingMap.remove(lastShooterDistance);
        return true;
    }

    public void clearAllMeasurements() {
        measurements.clear();
        interpolatingMap.clear();
        measurementHistory.clear();
    }

    public boolean save(int slot, String description) {
        
        try {
            
            ObjectMapper mapper = new ObjectMapper();

            File file = getSlotFile(slot);


            List<CalibrationRecord> serializedMeasurements = new ArrayList<>();
            measurements.forEach((shooterDistance, rpms) -> serializedMeasurements.add(new CalibrationRecord(shooterDistance, rpms.getX(), rpms.getY())));
            serializedMeasurements.sort(Comparator.comparingDouble(record -> record.distance));

            ShooterCalib calib = new ShooterCalib(description, serializedMeasurements);

            mapper.writerWithDefaultPrettyPrinter()
                    .writeValue(file, calib);

            this.slot = slot;
            this.description = description;
            
            return true;

        } catch (Exception ex) {
            ex.printStackTrace();
            return false;
        }
    }

    public String getDescription()
    {
        return description;
    }

    public int getSlot()
    {
        return slot;
    }

    public Map<Double, Translation2d> getMeasurements()
    {
        return Collections.unmodifiableMap(measurements);
    }
    
    public static ShooterCalibration loadFromSlot(int slot)
    {
        try {

            ObjectMapper mapper = new ObjectMapper();
            
            File file = getSlotFile(slot);

            if(!file.exists()) return null;

            ShooterCalibration calib = new ShooterCalibration();

            ShooterCalib slotData = mapper.readValue(file, ShooterCalib.class);

            slotData.getRecords().forEach(record -> calib.addMeasurement(record.distance, record.upperRPM, record.lowerRPM));

            calib.description = slotData.description;
            calib.slot = slot;

            return calib;

        } catch (Exception ex) {
            ex.printStackTrace();
            return null;
        }
    }

    public static boolean selectCalibrationSlot(int slot) {
        
        if(slot < 0) return false;

        try {
            ObjectMapper mapper = new ObjectMapper();
            
            File file = getGeneralConfigFile();

            ShooterConfig config = new ShooterConfig(slot);

            mapper.writerWithDefaultPrettyPrinter().writeValue(file, config);

            return true;
        } catch (Exception ex) {
            ex.printStackTrace();
            return false;
        }
    }

    public static int getSelectedCalibrationSlot() {
        try {

            ObjectMapper mapper = new ObjectMapper();
            
            File file = getGeneralConfigFile();

            if(!file.exists()) return DEFAULT_SLOT;

            ShooterConfig config = mapper.readValue(file, ShooterConfig.class);
            
            return config.slot;

        } catch (Exception ex) {
            ex.printStackTrace();
            return -1;
        }
    }

    // Privs

    private static File getSlotFile(int slot) {
        File file = new File(
                Filesystem.getOperatingDirectory(),
                String.format("shootercalib/shootercalib%d.json", slot));

        file.getParentFile().mkdirs();

        return file;
    }

    private static File getGeneralConfigFile() {
        File file = new File(
                Filesystem.getOperatingDirectory(),
                "shootercalib/config.json");

        file.getParentFile().mkdirs();

        return file;
    }
}