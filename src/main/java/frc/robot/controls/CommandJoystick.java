package frc.robot.controls;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.controls.util.AxisUtil;

public class CommandJoystick extends CommandGenericHID {
    private static final double STICK_EXPO = 0.0;

    private final JoystickOptions options;
    private final PS5Controller ps5Controller;
    private final XboxController xboxController;
    private final DoublePublisher leftXPub;
    private final DoublePublisher leftYPub;
    private final DoublePublisher rightXPub;
    private final DoublePublisher rightYPub;
    private final DoublePublisher leftXRawPub;
    private final DoublePublisher leftYRawPub;
    private final DoublePublisher rightXRawPub;
    private final DoublePublisher rightYRawPub;
    private final DoublePublisher l2Pub;
    private final DoublePublisher r2Pub;
    private final DoublePublisher l2RawPub;
    private final DoublePublisher r2RawPub;
    private final BooleanPublisher btnDownPub;
    private final BooleanPublisher btnUpPub;
    private final BooleanPublisher btnLeftPub;
    private final BooleanPublisher btnRightPub;
    private final BooleanPublisher l1Pub;
    private final BooleanPublisher r1Pub;
    private final IntegerPublisher povPub;
    private final StringPublisher controllerTypePub;

    public CommandJoystick(JoystickOptions options) {
        super(Constants.OperatorConstants.kDriverControllerPort);
        this.options = options;
        if (options == JoystickOptions.DualSense) {
            this.ps5Controller = new PS5Controller(Constants.OperatorConstants.kDriverControllerPort);
            this.xboxController = null;
        } else {
            this.ps5Controller = null;
            this.xboxController = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
        }

        NetworkTable table = NetworkTableInstance.getDefault().getTable("Controls/Driver");
        controllerTypePub = table.getStringTopic("ControllerType").publish();
        leftXPub = table.getDoubleTopic("LeftX").publish();
        leftYPub = table.getDoubleTopic("LeftY").publish();
        rightXPub = table.getDoubleTopic("RightX").publish();
        rightYPub = table.getDoubleTopic("RightY").publish();
        leftXRawPub = table.getDoubleTopic("LeftXRaw").publish();
        leftYRawPub = table.getDoubleTopic("LeftYRaw").publish();
        rightXRawPub = table.getDoubleTopic("RightXRaw").publish();
        rightYRawPub = table.getDoubleTopic("RightYRaw").publish();
        l2Pub = table.getDoubleTopic("L2").publish();
        r2Pub = table.getDoubleTopic("R2").publish();
        l2RawPub = table.getDoubleTopic("L2Raw").publish();
        r2RawPub = table.getDoubleTopic("R2Raw").publish();
        btnDownPub = table.getBooleanTopic("BtnDown").publish();
        btnUpPub = table.getBooleanTopic("BtnUp").publish();
        btnLeftPub = table.getBooleanTopic("BtnLeft").publish();
        btnRightPub = table.getBooleanTopic("BtnRight").publish();
        l1Pub = table.getBooleanTopic("L1").publish();
        r1Pub = table.getBooleanTopic("R1").publish();
        povPub = table.getIntegerTopic("Pov").publish();
    }

    public double getLeftX() {
        return processStickAxis(rawLeftX(), DriveConstants.JOYDeadzone_Y);
    }

    public double getLeftY() {
        return processStickAxis(-rawLeftY(), DriveConstants.JOYDeadzone_X);
    }

    public double getRightX() {
        return processStickAxis(rawRightX(), DriveConstants.JOYDeadzone_Rot);
    }

    public double getRightY() {
        return processStickAxis(-rawRightY(), DriveConstants.JOYDeadzone_X);
    }

    public double getL2() {
        return normalizeTrigger(rawL2());
    }

    public double getR2() {
        return normalizeTrigger(rawR2());
    }

    public Trigger getL1() {
        return new Trigger(() -> isDualSense()
            ? ps5Controller.getL1Button()
            : xboxController.getLeftBumper());
    }

    public Trigger getR1() {
        return new Trigger(() -> isDualSense()
            ? ps5Controller.getR1Button()
            : xboxController.getRightBumper());
    }

    public Trigger getBtnDown() {
        return new Trigger(() -> isDualSense()
            ? ps5Controller.getCrossButton()
            : xboxController.getAButton());
    }

    public Trigger getBtnUp() {
        return new Trigger(() -> isDualSense()
            ? ps5Controller.getTriangleButton()
            : xboxController.getYButton());
    }

    public Trigger getBtnLeft() {
        return new Trigger(() -> isDualSense()
            ? ps5Controller.getSquareButton()
            : xboxController.getXButton());
    }

    public Trigger getBtnRight() {
        return new Trigger(() -> isDualSense()
            ? ps5Controller.getCircleButton()
            : xboxController.getBButton());
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
        return new Trigger(() -> getPov() == -1);
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

    public void updateNetworkTables() {
        double leftXRaw = rawLeftX();
        double leftYRaw = rawLeftY();
        double rightXRaw = rawRightX();
        double rightYRaw = rawRightY();
        double l2Raw = rawL2();
        double r2Raw = rawR2();

        controllerTypePub.set(options.name());
        leftXPub.set(getLeftX());
        leftYPub.set(getLeftY());
        rightXPub.set(getRightX());
        rightYPub.set(getRightY());
        leftXRawPub.set(leftXRaw);
        leftYRawPub.set(leftYRaw);
        rightXRawPub.set(rightXRaw);
        rightYRawPub.set(rightYRaw);
        l2Pub.set(normalizeTrigger(l2Raw));
        r2Pub.set(normalizeTrigger(r2Raw));
        l2RawPub.set(l2Raw);
        r2RawPub.set(r2Raw);
        btnDownPub.set(isDualSense()
            ? ps5Controller.getCrossButton()
            : xboxController.getAButton());
        btnUpPub.set(isDualSense()
            ? ps5Controller.getTriangleButton()
            : xboxController.getYButton());
        btnLeftPub.set(isDualSense()
            ? ps5Controller.getSquareButton()
            : xboxController.getXButton());
        btnRightPub.set(isDualSense()
            ? ps5Controller.getCircleButton()
            : xboxController.getBButton());
        l1Pub.set(isDualSense()
            ? ps5Controller.getL1Button()
            : xboxController.getLeftBumper());
        r1Pub.set(isDualSense()
            ? ps5Controller.getR1Button()
            : xboxController.getRightBumper());
        povPub.set(getPov());
    }

    private Trigger povAt(int angle) {
        return new Trigger(() -> getPov() == angle);
    }

    private static double processStickAxis(double value, double deadband) {
        double processed = AxisUtil.applyDeadband(value, deadband);
        processed = AxisUtil.applyExpo(processed, STICK_EXPO);
        return AxisUtil.clamp(processed, -1.0, 1.0);
    }

    private boolean isDualSense() {
        return options == JoystickOptions.DualSense;
    }

    private int getPov() {
        return isDualSense() ? ps5Controller.getPOV() : xboxController.getPOV();
    }

    private double rawLeftX() {
        return isDualSense() ? ps5Controller.getLeftX() : xboxController.getLeftX();
    }

    private double rawLeftY() {
        return isDualSense() ? ps5Controller.getLeftY() : xboxController.getLeftY();
    }

    private double rawRightX() {
        return isDualSense() ? ps5Controller.getRightX() : xboxController.getRightX();
    }

    private double rawRightY() {
        return isDualSense() ? ps5Controller.getRightY() : xboxController.getRightY();
    }

    private double rawL2() {
        return isDualSense() ? ps5Controller.getL2Axis() : xboxController.getLeftTriggerAxis();
    }

    private double rawR2() {
        return isDualSense() ? ps5Controller.getR2Axis() : xboxController.getRightTriggerAxis();
    }

    private static double normalizeTrigger(double value) {
        if (value < 0.0) {
            return AxisUtil.normalizeTriggerMinusOneToOneToZeroToOne(value);
        }
        return AxisUtil.clamp(value, 0.0, 1.0);
    }
}
