package frc.robot.shooter;

import java.io.File;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.Filesystem;

public class ShooterCalibration {

    InterpolatingTreeMap<Double, Double> interpolatingMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(),
            Interpolator.forDouble());

    private HashMap<Double, Double> measurements = new HashMap<Double, Double>();
    private String description;
    private int slot;

    private record ShooterConfig(int slot) {
    };

    
    private record ShooterCalib(String description, HashMap<Double, Double> knownValues) {
    };

    private static final int DEFAULT_SLOT = 0;

    public ShooterCalibration() {
        
    }

    public void addMeasurement(double distance, double rpm) {
        measurements.putIfAbsent(distance, rpm);
        interpolatingMap.put(distance, rpm);
    }

    public boolean save(int slot, String description) {
        
        try {
            
            ObjectMapper mapper = new ObjectMapper();

            File file = getSlotFile(slot);


            ShooterCalib calib = new ShooterCalib(description, measurements);

            mapper.writerWithDefaultPrettyPrinter()
                    .writeValue(file, calib);

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

    public Map<Double, Double> getMeasurements()
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

            ShooterCalib calibObj = mapper.readValue(file, ShooterCalib.class);

            calib.measurements = calibObj.knownValues;

            Double[] measurementKeys = calib.measurements.keySet().toArray(new Double[0]);

            for(int i = 0; i<calib.measurements.size(); i++)
            {
                calib.interpolatingMap.put(measurementKeys[i], calib.measurements.get(measurementKeys[i]));
            }

            calib.description = calibObj.description;
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
