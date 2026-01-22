package frc.robot.drivetrain.swerve.commands;

import java.util.Arrays;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
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
    final Supplier<Double> suppX, suppY, suppZ;
    final CommandXboxController joy;

    boolean xZero, yZero, zZero = true;

    public SwerveJoystickDriveCommand(
        SwerveSubsystem swerve,
        CommandXboxController joy,
        Supplier<Double> speedSuppX,
        Supplier<Double> speedSuppY,
        Supplier<Double> speedSuppRot) {

        this.swerve = swerve;
        this.joy = joy;
        this.suppX = speedSuppX;
        this.suppY = speedSuppY;
        this.suppZ = speedSuppRot;

        addRequirements(this.swerve);
    }
    
    @Override
    public void execute() {
        double xpercent = suppX.get();
        double ypercent = suppY.get();
        double zpercent = suppZ.get();

        xpercent = MathUtil.applyDeadband(xpercent, DriveConstants.JOYDeadzone_X);
        ypercent = MathUtil.applyDeadband(ypercent, DriveConstants.JOYDeadzone_Y);
        zpercent = MathUtil.applyDeadband(zpercent, DriveConstants.JOYDeadzone_Rot);

        xpercent = MathUtil.clamp(xpercent, -1, 1);
        ypercent = MathUtil.clamp(ypercent, -1, 1);
        zpercent = MathUtil.clamp(zpercent, -1, 1);

        ChassisSpeeds focs = swerve.getTargetFieldRelativeSpeeds();
        
        double targetSpeeds[] = new double[] {
            xpercent * DriveConstants.MaxDriveSpeed,
            ypercent * DriveConstants.MaxDriveSpeed,
            zpercent * DriveConstants.MaxRotSpeed
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
}
