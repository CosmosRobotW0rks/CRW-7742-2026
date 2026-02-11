package frc.robot.controls.util;

public final class AxisUtil {
    private AxisUtil() {}

    public static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) <= deadband) {
            return 0.0;
        }
        return (value - Math.copySign(deadband, value)) / (1.0 - deadband);
    }

    public static double applyExpo(double value, double expo) {
        if (expo <= 0.0) {
            return value;
        }
        double clamped = clamp(value, -1.0, 1.0);
        return clamped * (1.0 - expo) + Math.pow(clamped, 3) * expo;
    }

    public static double normalizeTriggerMinusOneToOneToZeroToOne(double value) {
        return clamp((value + 1.0) / 2.0, 0.0, 1.0);
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
