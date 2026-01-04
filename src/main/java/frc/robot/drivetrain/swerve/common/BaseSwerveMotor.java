package frc.robot.drivetrain.swerve.common;

public interface BaseSwerveMotor {
    public boolean init(SwerveMotorConfig config);
    
    public void simulationPeriodic();
}
