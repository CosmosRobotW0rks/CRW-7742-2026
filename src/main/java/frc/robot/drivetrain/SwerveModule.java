package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.Meters;

import java.io.ObjectInputFilter.Config;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.drivetrain.Common.BaseSwerveAngleMotor;
import frc.robot.drivetrain.Common.BaseSwerveDriveMotor;
import frc.robot.drivetrain.Common.SwerveModuleConfiguration;
import frc.robot.drivetrain.Common.SwerveMotorConfig;

public class SwerveModule {
    public SwerveModule() {
    }

    SwerveModuleConfiguration moduleConfig;

    public void init(SwerveModuleConfiguration config) {
        this.moduleConfig = config;
    }

    public void stop()
    {
        moduleConfig.driveMotor().stop();
        moduleConfig.angleMotor().stop();
    }

    // GETTERS

    public SwerveModuleState GetState() {
        return new SwerveModuleState(moduleConfig.driveMotor().getCurrentVelocity(),
                moduleConfig.angleMotor().getCurrentAngle());
    }

    public SwerveModulePosition GetPosition() {
        return new SwerveModulePosition(Meters.of(moduleConfig.driveMotor().getTotalDistance()),
                moduleConfig.angleMotor().getCurrentAngle());
    }

    // SETTERS

    public void SetTargetState(SwerveModuleState state) {
        optimizeState(state, false);

        moduleConfig.angleMotor().setTargetAngle(state.angle);
        moduleConfig.driveMotor().setTargetVelocity(state.speedMetersPerSecond);
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
