package frc.robot.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ShooterSubsystem extends SubsystemBase {
    
  private final SparkMax m_motor = new SparkMax(5, MotorType.kBrushless);
  private final SparkClosedLoopController m_pidController = m_motor.getClosedLoopController();
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  
  public ShooterSubsystem()
  {
    com.revrobotics.spark.config.SparkMaxConfig config = new com.revrobotics.spark.config.SparkMaxConfig();
    config.smartCurrentLimit(40);

    config.closedLoop.pid(0.01, 0, 0);
    
    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  
  public Command shootCommand()
  {
    return Commands.run(() -> {
        m_pidController.setSetpoint(3000,ControlType.kVelocity);
    }, this).finallyDo(() -> m_pidController.setSetpoint(0, ControlType.kVelocity));
  }


  public Trigger isShooterAtTargetRPM()
  {
    return new Trigger(() -> m_encoder.getVelocity() == 3000);
  }



}