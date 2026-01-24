package frc.robot.input;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;

public class DriveInputSelector {
    private final CommandXboxController controller;
    private final Supplier<Double> defaultX;
    private final Supplier<Double> defaultY;
    private final Supplier<Double> defaultRot;
    private final double[] cachedPercents = new double[3];

    public DriveInputSelector(
            CommandXboxController controller,
            Supplier<Double> defaultX,
            Supplier<Double> defaultY,
            Supplier<Double> defaultRot) {
        this.controller = controller;
        this.defaultX = defaultX;
        this.defaultY = defaultY;
        this.defaultRot = defaultRot;
    }

    public double[] getDrivePercents() {
        if (OperatorConstants.isDualsense) {
            cachedPercents[0] = -controller.getHID().getRawAxis(OperatorConstants.DualSenseAxisLeftY);
            cachedPercents[1] = -controller.getHID().getRawAxis(OperatorConstants.DualSenseAxisLeftX);
            cachedPercents[2] = -controller.getHID().getRawAxis(OperatorConstants.DualSenseAxisRightX);
        } else {
            cachedPercents[0] = defaultX.get();
            cachedPercents[1] = defaultY.get();
            cachedPercents[2] = defaultRot.get();
        }

        return cachedPercents;
    }
}
