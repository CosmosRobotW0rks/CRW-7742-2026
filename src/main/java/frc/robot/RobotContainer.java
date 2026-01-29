// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.auto.AutoHelper;
import frc.robot.auto.autoCommands.ApproachPoseCommand;
import frc.robot.auto.autoCommands.ApproachPoseCommand.ApproachPoseConfiguration;
import frc.robot.drivetrain.VisionSubsystem;
//import frc.robot.drivetrain.VisionSubsystem;
import frc.robot.drivetrain.swerve.SwerveSubsystem;
import frc.robot.drivetrain.swerve.commands.SwerveJoystickDriveCommand;
import frc.robot.shooter.ShooterCalibration;
import frc.robot.shooter.ShooterCalibratorSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.utils.Logging;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


// Merhaba

public class RobotContainer {

  private final CommandXboxController joystick = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SwerveSubsystem swerveSubsystem;
  private final VisionSubsystem visionSubsystem;
  
  /* 
  private final ShooterSubsystem shooterSubsystem;
  private final ShooterCalibratorSubsystem shooterCalibratorSubsystem;
*/

  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem();

    /* 
    shooterSubsystem = new ShooterSubsystem();
    shooterCalibratorSubsystem = new ShooterCalibratorSubsystem(shooterSubsystem);
*/
    visionSubsystem = new VisionSubsystem(swerveSubsystem);
    
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

    joystick.a().whileTrue(AutoHelper.GetClimbCommand(swerveSubsystem));

    joystick.b().whileTrue(new ApproachPoseCommand(swerveSubsystem, new ApproachPoseConfiguration(1.55,3.55)));

/* 
    joystick.povDown().onTrue(shooterCalibratorSubsystem.IncreaseRPMCommand(-100));
    joystick.povUp().onTrue(shooterCalibratorSubsystem.IncreaseRPMCommand(100));

    joystick.b().toggleOnTrue(shooterSubsystem.prepareShooterCommand());

    joystick.a().and(shooterSubsystem.isReadyToShoot()).whileTrue(shooterSubsystem.FeedCommand());   
    */
    
  }

  double time = 0;

  public Command getAutonomousCommand() {

    return Commands.runOnce(() -> time = Timer.getFPGATimestamp()).andThen(new PathPlannerAuto("TESTAUTO")).andThen(() ->{
      System.out.println("AUTO TIME: " + (Timer.getFPGATimestamp() - time));
    }).finallyDo(() -> swerveSubsystem.Stop());
  }

}
