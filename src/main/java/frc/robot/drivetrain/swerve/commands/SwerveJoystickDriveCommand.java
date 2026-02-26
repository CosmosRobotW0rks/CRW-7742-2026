package frc.robot.drivetrain.swerve.commands;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.controls.CommandJoystick;
import frc.robot.drivetrain.swerve.SwerveSubsystem;

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

    boolean xZero, yZero, zZero = true;

    public SwerveJoystickDriveCommand(
        SwerveSubsystem swerve,
        DoubleSupplier sup_x,
        DoubleSupplier sup_y,
        DoubleSupplier sup_z) {

        this.swerve = swerve;

        this.supX = sup_x;
        this.supY = sup_y;
        this.supZ = sup_z;

        addRequirements(this.swerve);
    }
    
    @Override
    public void execute() {
<<<<<<< Updated upstream
        double xpercent = supX.getAsDouble();
        double ypercent = supY.getAsDouble();
        double zpercent = supZ.getAsDouble();
=======

        double xpercent = suppX.get();
        double ypercent = suppY.get();
        double zpercent = suppZ.get();
>>>>>>> Stashed changes

        if (DriveConstants.SquareInputs) {
            xpercent = squarePreserveSign(xpercent);
            ypercent = squarePreserveSign(ypercent);
            zpercent = squarePreserveSign(zpercent);
        }

<<<<<<< Updated upstream
        if (Math.abs(zpercent) < DriveConstants.JOYDeadzone_Rot) {
            zpercent = 0.0;
        }

        xpercent = clamp(xpercent, -1, 1);
        ypercent = clamp(ypercent, -1, 1);
        zpercent = clamp(zpercent, -1, 1);
=======
>>>>>>> Stashed changes

        double boostMultiplier = 1.0 + (joy.getRightTriggerAxis() * 0.5);


        double targetSpeeds[] = new double[] {
            xpercent * DriveConstants.MaxDriveSpeed * boostMultiplier,
            ypercent * DriveConstants.MaxDriveSpeed * boostMultiplier,
            zpercent * DriveConstants.MaxRotSpeed * boostMultiplier
        };

<<<<<<< Updated upstream
        //applyFilter(targetSpeeds, new double[]{focs.vxMetersPerSecond, focs.vyMetersPerSecond, focs.omegaRadiansPerSecond});
=======
>>>>>>> Stashed changes

        ChassisSpeeds currentSpeeds = swerve.getTargetFieldRelativeSpeeds();
        applyFilter(targetSpeeds, new double[]{
            currentSpeeds.vxMetersPerSecond, 
            currentSpeeds.vyMetersPerSecond, 
            currentSpeeds.omegaRadiansPerSecond
        });

        if(xZero && yZero && zZero && Arrays.stream(targetSpeeds).allMatch(val -> val == 0)) return;

        xZero = targetSpeeds[0] == 0;
        yZero = targetSpeeds[1] == 0;
        zZero = targetSpeeds[2] == 0;

<<<<<<< Updated upstream
        //SmartDashboard.putNumberArray("CommOutArr", new double[] {targetSpeeds[0], targetSpeeds[1], targetSpeeds[2]});

        swerve.setTargetFieldRelativeSpeeds(targetSpeeds[0], targetSpeeds[1], targetSpeeds[2]);
=======
        ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds(targetSpeeds[0], targetSpeeds[1], targetSpeeds[2]);
        swerve.setTargetFieldRelativeSpeeds(targetChassisSpeeds);
>>>>>>> Stashed changes
    }

    void applyFilter(double[] targetSpeeds, double[] prevSpeeds) {
        for(int i = 0; i < 3; i++) {
            double incSpeed = accelFilters[i].calculate(targetSpeeds[i]);
            double descSpeed = deccelFilters[i].calculate(targetSpeeds[i]);

            if(Math.abs(prevSpeeds[i]) < Math.abs(targetSpeeds[i])) {
                targetSpeeds[i] = incSpeed;
            } else {
                targetSpeeds[i] = descSpeed;
            }
        }
    }

<<<<<<< Updated upstream
    private static double squarePreserveSign(double value) {
        return Math.copySign(value * value, value);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
=======
    @Override
    public void end(boolean interrupted) {
        swerve.setTargetFieldRelativeSpeeds(new ChassisSpeeds(0, 0, 0));
    }
}
>>>>>>> Stashed changes
