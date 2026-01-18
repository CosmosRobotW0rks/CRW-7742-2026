// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
//import frc.robot.drivetrain.VisionSubsystem;
import frc.robot.drivetrain.swerve.SwerveSubsystem;
import frc.robot.drivetrain.swerve.commands.SwerveJoystickDriveCommand;
import frc.robot.shooter.ShooterCalibration;
import frc.robot.shooter.ShooterCalibratorSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.utils.Logging;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController joystick = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SwerveSubsystem swerveSubsystem;
  //private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  private final ShooterSubsystem shooterSubsystem;
  private final ShooterCalibratorSubsystem shooterCalibratorSubsystem;

  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    shooterCalibratorSubsystem = new ShooterCalibratorSubsystem(shooterSubsystem);
    
    configureBindings();

    Logging.infoMsg("Init Complete","Initialized subsystems");
    
  }

  private void configureBindings() {

    // TODO: Are the joystick directions (signs) correct?
    
    swerveSubsystem.setDefaultCommand(new SwerveJoystickDriveCommand(
        swerveSubsystem,
        joystick,
        () -> -joystick.getLeftY(),
        () -> -joystick.getLeftX(),
        () -> -joystick.getRightX()));


    joystick.povDown().onTrue(shooterCalibratorSubsystem.IncreaseRPMCommand(-100));
    joystick.povUp().onTrue(shooterCalibratorSubsystem.IncreaseRPMCommand(100));

    joystick.b().toggleOnTrue(shooterSubsystem.prepareShooterCommand());

    joystick.a().and(shooterSubsystem.isReadyToShoot()).whileTrue(shooterSubsystem.FeedCommand());
         
    
    
  }

  double time = 0;

  public Command getAutonomousCommand() {

    return Commands.runOnce(() -> time = Timer.getFPGATimestamp()).andThen(new PathPlannerAuto("New Auto")).andThen(() ->{
      System.out.println("AUTO TIME: " + (Timer.getFPGATimestamp() - time));
    }).finallyDo(() -> swerveSubsystem.Stop());
  }

}
