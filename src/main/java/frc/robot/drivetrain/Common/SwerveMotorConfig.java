package frc.robot.drivetrain.Common;


public record SwerveMotorConfig(int canID, double gearRatio, double wheelRadius, double peakVoltage, double kP, double kI, double kD, double kV) {}
