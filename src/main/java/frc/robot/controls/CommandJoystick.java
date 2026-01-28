package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.controls.mapping.DualSenseMap;
import frc.robot.controls.mapping.LogitechMap;
import frc.robot.controls.util.AxisUtil;

public class CommandJoystick extends CommandGenericHID {
    private static final double STICK_EXPO = 0.0;

    private final Joystick joystick;
    private final MapConfig map;

    public CommandJoystick(JoystickOptions options) {
        super(Constants.OperatorConstants.kDriverControllerPort);
        this.joystick = new Joystick(Constants.OperatorConstants.kDriverControllerPort);
        this.map = MapConfig.from(options);
    }

    public double getLeftX() {
        return processStickAxis(joystick.getRawAxis(map.axisLeftX), DriveConstants.JOYDeadzone_Y);
    }

    public double getLeftY() {
        return processStickAxis(joystick.getRawAxis(map.axisLeftY), DriveConstants.JOYDeadzone_X);
    }

    public double getRightX() {
        return processStickAxis(joystick.getRawAxis(map.axisRightX), DriveConstants.JOYDeadzone_Rot);
    }

    public double getRightY() {
        return processStickAxis(joystick.getRawAxis(map.axisRightY), DriveConstants.JOYDeadzone_X);
    }

    public double getL2() {
        return AxisUtil.normalizeTriggerMinusOneToOneToZeroToOne(
            joystick.getRawAxis(map.axisL2));
    }

    public double getR2() {
        return AxisUtil.normalizeTriggerMinusOneToOneToZeroToOne(
            joystick.getRawAxis(map.axisR2));
    }

    public Trigger getL1() {
        return button(map.buttonL1);
    }

    public Trigger getR1() {
        return button(map.buttonR1);
    }

    public Trigger getBtnDown() {
        return button(map.buttonDown);
    }

    public Trigger getBtnUp() {
        return button(map.buttonUp);
    }

    public Trigger getBtnLeft() {
        return button(map.buttonLeft);
    }

    public Trigger getBtnRight() {
        return button(map.buttonRight);
    }

    public Trigger getX() {
        return getBtnDown();
    }

    public Trigger getO() {
        return getBtnRight();
    }

    public Trigger getSquare() {
        return getBtnLeft();
    }

    public Trigger getTriangle() {
        return getBtnUp();
    }

    public Trigger getPovCenter() {
        return new Trigger(() -> joystick.getPOV() == -1);
    }

    public Trigger getPovDown() {
        return povAt(180);
    }

    public Trigger getPovUp() {
        return povAt(0);
    }

    public Trigger getPovLeft() {
        return povAt(270);
    }

    public Trigger getPovRight() {
        return povAt(90);
    }

    public Trigger getPovDownLeft() {
        return povAt(225);
    }

    public Trigger getPovDownRight() {
        return povAt(135);
    }

    public Trigger getPovUpLeft() {
        return povAt(315);
    }

    public Trigger getPovUpRight() {
        return povAt(45);
    }

    private Trigger povAt(int angle) {
        return new Trigger(() -> joystick.getPOV() == angle);
    }

    private static double processStickAxis(double value, double deadband) {
        double processed = AxisUtil.applyDeadband(value, deadband);
        processed = AxisUtil.applyExpo(processed, STICK_EXPO);
        return AxisUtil.clamp(processed, -1.0, 1.0);
    }

    private static final class MapConfig {
        final int axisLeftX;
        final int axisLeftY;
        final int axisRightX;
        final int axisRightY;
        final int axisL2;
        final int axisR2;
        final int buttonL1;
        final int buttonR1;
        final int buttonDown;
        final int buttonUp;
        final int buttonLeft;
        final int buttonRight;

        private MapConfig(
            int axisLeftX,
            int axisLeftY,
            int axisRightX,
            int axisRightY,
            int axisL2,
            int axisR2,
            int buttonL1,
            int buttonR1,
            int buttonDown,
            int buttonUp,
            int buttonLeft,
            int buttonRight) {
            this.axisLeftX = axisLeftX;
            this.axisLeftY = axisLeftY;
            this.axisRightX = axisRightX;
            this.axisRightY = axisRightY;
            this.axisL2 = axisL2;
            this.axisR2 = axisR2;
            this.buttonL1 = buttonL1;
            this.buttonR1 = buttonR1;
            this.buttonDown = buttonDown;
            this.buttonUp = buttonUp;
            this.buttonLeft = buttonLeft;
            this.buttonRight = buttonRight;
        }

        static MapConfig from(JoystickOptions options) {
            switch (options) {
                case DualSense:
                    return new MapConfig(
                        DualSenseMap.AXIS_LEFT_X,
                        DualSenseMap.AXIS_LEFT_Y,
                        DualSenseMap.AXIS_RIGHT_X,
                        DualSenseMap.AXIS_RIGHT_Y,
                        DualSenseMap.AXIS_L2,
                        DualSenseMap.AXIS_R2,
                        DualSenseMap.BUTTON_L1,
                        DualSenseMap.BUTTON_R1,
                        DualSenseMap.BUTTON_DOWN,
                        DualSenseMap.BUTTON_UP,
                        DualSenseMap.BUTTON_LEFT,
                        DualSenseMap.BUTTON_RIGHT);
                case Logitech:
                    return new MapConfig(
                        LogitechMap.AXIS_LEFT_X,
                        LogitechMap.AXIS_LEFT_Y,
                        LogitechMap.AXIS_RIGHT_X,
                        LogitechMap.AXIS_RIGHT_Y,
                        LogitechMap.AXIS_L2,
                        LogitechMap.AXIS_R2,
                        LogitechMap.BUTTON_L1,
                        LogitechMap.BUTTON_R1,
                        LogitechMap.BUTTON_DOWN,
                        LogitechMap.BUTTON_UP,
                        LogitechMap.BUTTON_LEFT,
                        LogitechMap.BUTTON_RIGHT);
                default:
                    throw new IllegalStateException("Unsupported controller: " + options);
            }
        }
    }
}
