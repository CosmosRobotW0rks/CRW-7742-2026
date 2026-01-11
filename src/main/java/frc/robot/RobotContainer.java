// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
//import frc.robot.drivetrain.VisionSubsystem;
import frc.robot.drivetrain.swerve.SwerveSubsystem;
import frc.robot.drivetrain.swerve.commands.SwerveJoystickDriveCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController joystick = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  //private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    // TODO: Are the joystick directions (signs) correct?
    swerveSubsystem.setDefaultCommand(new SwerveJoystickDriveCommand(
        swerveSubsystem,
        joystick,
        () -> -joystick.getLeftY(),
        () -> -joystick.getLeftX(),
        () -> -joystick.getRightX()));
    
    
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
