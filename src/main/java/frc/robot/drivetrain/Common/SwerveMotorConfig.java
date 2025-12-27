package frc.robot.drivetrain.Common;

import java.util.function.Supplier;


public record SwerveMotorConfig(int canID, double gearRatio, double wheelRadius, double peakVoltage, double kP, double kI, double kD, double kV) {}
