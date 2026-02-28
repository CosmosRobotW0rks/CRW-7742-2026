package frc.robot.subsystems.health;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.swerve.SwerveSubsystem;
import frc.robot.subsystems.drivetrain.swerve.common.SwerveModuleLocation;

public class HealthMonitoring extends SubsystemBase {

    private final NetworkTable table;
    private final DoublePublisher batteryVoltagePub;
    private final DoublePublisher totalCurrentPub;
    private final PowerDistribution pdp;
    private final SwerveSubsystem swerveSubsystem;

    private final BooleanSubscriber resetFLSub;
    private final BooleanSubscriber resetFRSub;
    private final BooleanSubscriber resetBLSub;
    private final BooleanSubscriber resetBRSub;
    private final BooleanSubscriber resetAllSub;

    public HealthMonitoring(PowerDistribution pdp, SwerveSubsystem swerveSubsystem) {
        this.pdp = pdp;
        this.swerveSubsystem = swerveSubsystem;

        table = NetworkTableInstance.getDefault().getTable("RobotHealth");

        batteryVoltagePub = table.getDoubleTopic("battery/voltage").publish();
        totalCurrentPub   = table.getDoubleTopic("pdp/total_current").publish();

        resetFLSub  = table.getBooleanTopic("commands/reset_FL").subscribe(false);
        resetFRSub  = table.getBooleanTopic("commands/reset_FR").subscribe(false);
        resetBLSub  = table.getBooleanTopic("commands/reset_BL").subscribe(false);
        resetBRSub  = table.getBooleanTopic("commands/reset_BR").subscribe(false);
        resetAllSub = table.getBooleanTopic("commands/reset_all").subscribe(false);
    }

    @Override
    public void periodic() {
        batteryVoltagePub.set(RobotController.getBatteryVoltage());
        totalCurrentPub.set(pdp.getTotalCurrent());
        handleCommands();
    }

    private void handleCommands() {
        if (resetFLSub.get()) {
            CommandScheduler.getInstance().schedule(swerveSubsystem.resetCANCoderOffsetCommand(SwerveModuleLocation.FRONT_LEFT));
            table.getEntry("commands/reset_FL").setBoolean(false);
        }
        if (resetFRSub.get()) {
            CommandScheduler.getInstance().schedule(swerveSubsystem.resetCANCoderOffsetCommand(SwerveModuleLocation.FRONT_RIGHT));
            table.getEntry("commands/reset_FR").setBoolean(false);
        }
        if (resetBLSub.get()) {
            CommandScheduler.getInstance().schedule(swerveSubsystem.resetCANCoderOffsetCommand(SwerveModuleLocation.BACK_LEFT));
            table.getEntry("commands/reset_BL").setBoolean(false);
        }
        if (resetBRSub.get()) {
            CommandScheduler.getInstance().schedule(swerveSubsystem.resetCANCoderOffsetCommand(SwerveModuleLocation.BACK_RIGHT));
            table.getEntry("commands/reset_BR").setBoolean(false);
        }
        if (resetAllSub.get()) {
            CommandScheduler.getInstance().schedule(swerveSubsystem.resetAllCANCoderOffsetsCommand());
            table.getEntry("commands/reset_all").setBoolean(false);
        }
    }
}