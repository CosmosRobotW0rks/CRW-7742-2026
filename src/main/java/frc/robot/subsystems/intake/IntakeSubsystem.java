package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.function.BooleanSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.utils.Logging;

public class IntakeSubsystem extends SubsystemBase {

    enum IntakeState {
        IDLE,
        CLOSED,
        OPEN
    };

    IntakeState state = IntakeState.IDLE;

    SparkMax rollerMotor;
    SparkMax angleMotor;

    public IntakeSubsystem() {

        rollerMotor = new SparkMax(IntakeConstants.IntakeRollerMotorCANID, MotorType.kBrushless);
        SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
        rollerMotor.configure(rollerMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        angleMotor = new SparkMax(IntakeConstants.IntakeAngleMotorCANID, MotorType.kBrushless);
        SparkMaxConfig angleMotorConfig = new SparkMaxConfig();
        angleMotorConfig.idleMode(IdleMode.kBrake);
        angleMotorConfig.closedLoop.p(IntakeConstants.IntakeAngleP_P);
        angleMotorConfig.closedLoop.maxOutput(IntakeConstants.IntakeAngleP_OutMax);
        angleMotorConfig.closedLoop.minOutput(-IntakeConstants.IntakeAngleP_OutMax);
        angleMotor.configure(angleMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putBoolean("Intake/IsOpen", isOpen());
        SmartDashboard.putBoolean("Intake/IsClosed", isClosed());
        SmartDashboard.putString("Intake/TargetState", state.toString());
        SmartDashboard.putNumber("Intake/Angle", -1);
    }

    public Command Toggle() {
        return Commands.deferredProxy(() -> {
            if (state == IntakeState.CLOSED)
                return Open();
            else
                return Close();
        });
    }

    public Command Open() {
        return Commands.race(Commands.run(() -> state = IntakeState.OPEN, this), Commands.waitUntil(() -> isOpen()));
    }

    public Command Close() {
        return Commands.race(Commands.run(() -> state = IntakeState.CLOSED, this),
                Commands.waitUntil(() -> isClosed()));
    }

    @Override
    public void periodic() {
        switch (state) {
            case IDLE:
                angleMotor.stopMotor();
                break;

            case CLOSED:
                angleMotor.getClosedLoopController().setSetpoint(0, ControlType.kPosition);
                break;

            case OPEN:
                angleMotor.getClosedLoopController().setSetpoint(IntakeConstants.Intake_TargetAngle,
                        ControlType.kPosition);
            default:
                break;
        }

        double rollerTargetRPM = state == IntakeState.OPEN ? IntakeConstants.IntakeRoller_TargetVelocity : 0;
        
        rollerMotor.getClosedLoopController().setSetpoint(rollerTargetRPM, ControlType.kVelocity);

        SmartDashboard.putBoolean("Intake/IsOpen", isOpen());
        SmartDashboard.putBoolean("Intake/IsClosed", isClosed());
        SmartDashboard.putString("Intake/TargetState", state.toString());
        SmartDashboard.putNumber("Intake/Angle", angleMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake/RollerVelocity", rollerMotor.getEncoder().getVelocity());
    }

    private boolean isOpen() {
        double targetAngle = IntakeConstants.Intake_TargetAngle;
        double currAngle = angleMotor.getEncoder().getPosition();

        double error = Math.abs(targetAngle - currAngle);

        return error < IntakeConstants.Intake_AngleTolerance;
    }

    private boolean isClosed() {
        double targetAngle = 0;
        double currAngle = angleMotor.getEncoder().getPosition();

        double error = Math.abs(targetAngle - currAngle);

        return error < IntakeConstants.Intake_AngleTolerance;
    }
}
