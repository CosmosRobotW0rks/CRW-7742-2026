package frc.robot.drivetrain.Common;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public interface BaseSwerveAngleMotor {

    public boolean init(SwerveMotorConfig config);

    public Rotation2d getCurrentAngle();
    public Rotation2d getTargetAngle();

    public void setCurrentAngle(Rotation2d angle);
    public void setTargetAngle(Rotation2d angle);

    public void stop();
}
