// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controls.CommandJoystick;
import frc.robot.controls.JoystickOptions;
import frc.robot.subsystems.auto.AutoHelper;
import frc.robot.subsystems.auto.AutoSubsystem;
import frc.robot.subsystems.auto.autoCommands.*;
import frc.robot.subsystems.auto.autoCommands.ApproachPoseCommand.ApproachPoseConfiguration;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drivetrain.VisionSubsystem;
import frc.robot.subsystems.drivetrain.swerve.SwerveSubsystem;
import frc.robot.subsystems.drivetrain.swerve.commands.SwerveJoystickDriveCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.neopixel.NeoPixelSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.AllianceUtils;
import frc.robot.utils.Logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;

public class RobotContainer {

  private final CommandJoystick driver = new CommandJoystick(JoystickOptions.DualSense);

  private final NeoPixelSubsystem neopixelSubsystem;

  private final SwerveSubsystem swerveSubsystem;
  
  @SuppressWarnings("unused")
  private final VisionSubsystem visionSubsystem;

  private final IntakeSubsystem intakeSubsystem;
  private final ClimbSubsystem climbSubsystem;

  private final ShooterSubsystem shooterSubsystem;

  private final AutoSubsystem autoSubsystem;

  //private final ShooterCalibratorSubsystem shooterCalibratorSubsystem;

  /*
   * private final ShooterSubsystem shooterSubsystem;
   * private final ShooterCalibratorSubsystem shooterCalibratorSubsystem;
   */

  public RobotContainer() {
    neopixelSubsystem = new NeoPixelSubsystem();
    

    swerveSubsystem = new SwerveSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    shooterSubsystem = new ShooterSubsystem(swerveSubsystem, neopixelSubsystem);
    //shooterCalibratorSubsystem = new ShooterCalibratorSubsystem(shooterSubsystem);
    visionSubsystem = new VisionSubsystem(swerveSubsystem, neopixelSubsystem);
    climbSubsystem = new ClimbSubsystem();

    autoSubsystem = new AutoSubsystem(swerveSubsystem, shooterSubsystem, climbSubsystem);

    configureBindings();

    Logging.infoMsg("Init Complete", "Initialized subsystems");

  }

  private void configureBindings() {

    swerveSubsystem.setDefaultCommand(new SwerveJoystickDriveCommand(
        swerveSubsystem,
        () -> driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> -driver.getRightX(),
        () -> driver.getR2()));

    
    driver.getBtnUp().onTrue(climbSubsystem.Toggle());

    driver.getL1().onTrue(intakeSubsystem.Toggle());
    driver.getR1().onTrue(intakeSubsystem.Toggle());

    driver.getPovUp().whileTrue(autoSubsystem.getAutoClimbCommand());


    driver.getBtnDown().whileTrue(shooterSubsystem.prepareAndShootCommand(false));
    
    new Trigger(() -> (driver.getL2() > 0.05))
    .whileTrue(AutoHelper.GetApproachHubCommand(swerveSubsystem, () -> swerveSubsystem.getRobotPose(), () -> driver.getL2())
    .andThen(shooterSubsystem.prepareAndShootCommand(true))    
    );
   
  }

  double time = 0;

  public Command getAutonomousCommand() {
    return autoSubsystem.getAutoCommand();
  }

  public void updateNetworkTables() {
    driver.updateNetworkTables();
  }

}
