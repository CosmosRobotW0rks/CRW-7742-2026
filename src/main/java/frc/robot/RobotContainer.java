// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.auto.AutoHelper;
import frc.robot.auto.autoCommands.ApproachPoseCommand;
import frc.robot.auto.autoCommands.ApproachPoseCommand.ApproachPoseConfiguration;
import frc.robot.controls.CommandJoystick;
import frc.robot.controls.JoystickOptions;
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

public class RobotContainer {

  private final CommandJoystick driver = new CommandJoystick(JoystickOptions.DualSense);

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
        driver));

    driver.getBtnDown().whileTrue(AutoHelper.GetClimbCommand(swerveSubsystem));

    driver.getBtnRight().whileTrue(new ApproachPoseCommand(swerveSubsystem, new ApproachPoseConfiguration(1.55,3.55)));

/* 
    driver.getPovDown().onTrue(shooterCalibratorSubsystem.IncreaseRPMCommand(-100));
    driver.getPovUp().onTrue(shooterCalibratorSubsystem.IncreaseRPMCommand(100));

    driver.getBtnRight().toggleOnTrue(shooterSubsystem.prepareShooterCommand());

    driver.getBtnDown().and(shooterSubsystem.isReadyToShoot()).whileTrue(shooterSubsystem.FeedCommand());   
    */
    
  }

  double time = 0;

  public Command getAutonomousCommand() {

    return Commands.runOnce(() -> time = Timer.getFPGATimestamp()).andThen(new PathPlannerAuto("TESTAUTO")).andThen(() ->{
      System.out.println("AUTO TIME: " + (Timer.getFPGATimestamp() - time));
    }).finallyDo(() -> swerveSubsystem.Stop());
  }

  public void updateNetworkTables() {
    driver.updateNetworkTables();
  }

}
