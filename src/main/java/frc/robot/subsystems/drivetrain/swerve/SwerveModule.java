package frc.robot.subsystems.drivetrain.swerve;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drivetrain.swerve.common.SwerveModuleConfiguration;

public class SwerveModule {
    public SwerveModule() {
    }

    private SwerveModuleConfiguration moduleConfig = null;

    public void init(SwerveModuleConfiguration config) {
        this.moduleConfig = config;
    }

    public void stop()
    {
        if(moduleConfig == null) return;
        
        moduleConfig.driveMotor().stop();
        moduleConfig.angleMotor().stop();
    }

    // GETTERS

    public SwerveModuleState getState() {
        if(moduleConfig == null) return new SwerveModuleState(0, Rotation2d.kZero);

        return new SwerveModuleState(moduleConfig.driveMotor().getCurrentVelocity(),
                moduleConfig.angleMotor().getCurrentAngle());
    }

    public SwerveModulePosition getPosition() {
        if(moduleConfig == null) return new SwerveModulePosition(0,Rotation2d.kZero);

        return new SwerveModulePosition(Meters.of(moduleConfig.driveMotor().getTotalDistance()),
                moduleConfig.angleMotor().getCurrentAngle());
    }

    public Rotation2d getAngle() {
        return moduleConfig.angleMotor().getCurrentAngle();
    }

    // SETTERS

    public void SetTargetState(SwerveModuleState state) {
        optimizeState(state, false);
        
        moduleConfig.angleMotor().setTargetAngle(state.angle);
        moduleConfig.driveMotor().setTargetVelocity(state.speedMetersPerSecond);
    }

    // SIMULATION

    public void simulationPeriodic() {
        if(moduleConfig == null) return;
        moduleConfig.driveMotor().simulationPeriodic();
        moduleConfig.angleMotor().simulationPeriodic();
    }


    // UTILS

    private void optimizeState(SwerveModuleState state, boolean forceForward) {
        double currentAngle = moduleConfig.angleMotor().getCurrentAngle().getRadians();
        double currentAngleClamped = currentAngle % (2.0 * Math.PI);
        currentAngleClamped += currentAngleClamped < 0 ? 2.0 * Math.PI : 0;

        double t = state.angle.getRadians();
        double t1 = getShortestRoute(currentAngle, t);
        double t2 = getShortestRoute(currentAngle, t > Math.PI ? t - Math.PI : t + Math.PI);

        double diff1 = Math.abs(t1 - currentAngle);
        double diff2 = Math.abs(t2 - currentAngle);

        if (!forceForward && diff1 > diff2) {
            state.angle = Rotation2d.fromRadians(t2);

        } else {
            state.angle = Rotation2d.fromRadians(t1);
        }
    }

    private double getShortestRoute(double a1, double a2) {
        double a1_clamped = a1 % (2.0 * Math.PI);
        a1_clamped += a1_clamped < 0 ? 2.0 * Math.PI : 0;

        double nt = a2 + a1 - a1_clamped;
        if (a2 - a1_clamped > Math.PI)
            nt -= 2.0 * Math.PI;
        if (a2 - a1_clamped < -Math.PI)
            nt += 2.0 * Math.PI;

        return nt;
    }
}
