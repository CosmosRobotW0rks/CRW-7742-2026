package frc.robot.subsystems.drivetrain.swerve.common;

import com.ctre.phoenix6.hardware.CANcoder;

public record SwerveModuleConfiguration(SwerveModuleLocation location, BaseSwerveAngleMotor angleMotor, BaseSwerveDriveMotor driveMotor, CANcoder cancoder) {}