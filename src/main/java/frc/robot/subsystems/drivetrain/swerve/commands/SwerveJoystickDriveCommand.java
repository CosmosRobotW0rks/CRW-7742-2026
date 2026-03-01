package frc.robot.subsystems.drivetrain.swerve.commands;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.controls.CommandJoystick;
import frc.robot.subsystems.drivetrain.swerve.SwerveSubsystem;

public class SwerveJoystickDriveCommand extends Command {

    final SlewRateLimiter[] accelFilters = new SlewRateLimiter[] {
            new SlewRateLimiter(DriveConstants.MaxDriveAccel),
            new SlewRateLimiter(DriveConstants.MaxDriveAccel),
            new SlewRateLimiter(DriveConstants.MaxRotAccel)
    };

    final SlewRateLimiter[] deccelFilters = new SlewRateLimiter[] {
            new SlewRateLimiter(DriveConstants.MaxDriveDeccel),
            new SlewRateLimiter(DriveConstants.MaxDriveDeccel),
            new SlewRateLimiter(DriveConstants.MaxRotDeccel)
    };

    final SwerveSubsystem swerve;

    final DoubleSupplier supX;
    final DoubleSupplier supY;
    final DoubleSupplier supZ;
    final DoubleSupplier supBoost;

    boolean xZero, yZero, zZero = true;

    public SwerveJoystickDriveCommand(
        SwerveSubsystem swerve,
        DoubleSupplier sup_x,
        DoubleSupplier sup_y,
        DoubleSupplier sup_z,
        DoubleSupplier sup_boost) {

        this.swerve = swerve;

        this.supX = sup_x;
        this.supY = sup_y;
        this.supZ = sup_z;
        this.supBoost = sup_boost;

        addRequirements(this.swerve);
    }
    
    @Override
    public void execute() {
        double xpercent = supX.getAsDouble();
        double ypercent = supY.getAsDouble();
        double zpercent = supZ.getAsDouble();

        if (DriveConstants.SquareInputs) {
            xpercent = squarePreserveSign(xpercent);
            ypercent = squarePreserveSign(ypercent);
            zpercent = squarePreserveSign(zpercent);
        }

        if (Math.abs(zpercent) < DriveConstants.JOYDeadzone_Rot) {
            zpercent = 0.0;
        }

        xpercent = clamp(xpercent, -1, 1);
        ypercent = clamp(ypercent, -1, 1);
        zpercent = clamp(zpercent, -1, 1);

        //ChassisSpeeds focs = swerve.getTargetFieldRelativeSpeeds();

        double driveBoostAddition = supBoost.getAsDouble() * (DriveConstants.MaxSpeedWithBoost - DriveConstants.MaxDriveSpeed);
        double maxDriveSpeedWithBoost = DriveConstants.MaxDriveSpeed + driveBoostAddition;

        double rotBoostAddition = supBoost.getAsDouble() * (DriveConstants.MaxRotSpeedWithBoost - DriveConstants.MaxRotSpeed);
        double maxRotSpeedWithBoost = DriveConstants.MaxRotSpeed + rotBoostAddition;

        
        double targetSpeeds[] = new double[] {
            xpercent * maxDriveSpeedWithBoost,
            ypercent * maxDriveSpeedWithBoost,
            zpercent * maxRotSpeedWithBoost
        };

        //applyFilter(targetSpeeds, new double[]{focs.vxMetersPerSecond, focs.vyMetersPerSecond, focs.omegaRadiansPerSecond});

        
        if(xZero && yZero && zZero && Arrays.stream(targetSpeeds).allMatch(x -> x == 0)) return;

        xZero = targetSpeeds[0] == 0;
        yZero = targetSpeeds[1] == 0;
        zZero = targetSpeeds[2] == 0;

        //SmartDashboard.putNumberArray("CommOutArr", new double[] {targetSpeeds[0], targetSpeeds[1], targetSpeeds[2]});

        swerve.setTargetFieldRelativeSpeeds(targetSpeeds[0], targetSpeeds[1], targetSpeeds[2]);
    }

    void applyFilter(double[] targetSpeeds, double[] prevSpeeds)
    {
        for(int i = 0; i<3; i++)
        {
            double incSpeed = accelFilters[i].calculate(targetSpeeds[i]);
            double descSpeed = deccelFilters[i].calculate(targetSpeeds[i]);

            if(prevSpeeds[i] < targetSpeeds[i]) targetSpeeds[i] = incSpeed;
            else targetSpeeds[i] = descSpeed;
        }
    }

    private static double squarePreserveSign(double value) {
        return Math.copySign(value * value, value);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
