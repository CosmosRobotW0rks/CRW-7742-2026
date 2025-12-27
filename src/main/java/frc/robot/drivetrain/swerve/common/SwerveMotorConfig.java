package frc.robot.drivetrain.swerve.common;


public record SwerveMotorConfig(int canID, double gearRatio, double wheelRadius, double peakVoltage, double kP, double kI, double kD, double kV) {}
