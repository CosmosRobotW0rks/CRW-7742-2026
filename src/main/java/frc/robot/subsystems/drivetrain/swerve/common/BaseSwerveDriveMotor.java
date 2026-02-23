package frc.robot.subsystems.drivetrain.swerve.common;

public interface BaseSwerveDriveMotor extends BaseSwerveMotor{

    public double getTotalDistance();
    public double getCurrentVelocity();
    public double getTargetVelocity();

    public void setTargetVelocity(double velocityMetersPerSecond);

    public void stop();
    
}