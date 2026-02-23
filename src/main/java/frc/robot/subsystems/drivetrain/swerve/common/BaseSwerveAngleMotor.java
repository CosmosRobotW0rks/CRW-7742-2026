package frc.robot.subsystems.drivetrain.swerve.common;

import edu.wpi.first.math.geometry.Rotation2d;

public interface BaseSwerveAngleMotor extends BaseSwerveMotor {

    public Rotation2d getCurrentAngle();
    public Rotation2d getTargetAngle();

    public void setCurrentAngle(Rotation2d angle);
    public void setTargetAngle(Rotation2d angle);

    public void stop();
}
