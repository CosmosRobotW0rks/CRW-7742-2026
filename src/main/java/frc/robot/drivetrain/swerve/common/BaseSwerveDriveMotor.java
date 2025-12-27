package frc.robot.drivetrain.swerve.common;

public interface BaseSwerveDriveMotor {

    public boolean init(SwerveMotorConfig config);

    public double getTotalDistance();
    public double getCurrentVelocity();
    public double getTargetVelocity();

    public void setTargetVelocity(double velocityMetersPerSecond);

    public void stop();
    
}