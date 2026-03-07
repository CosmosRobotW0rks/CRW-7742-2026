// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controls.CommandJoystick;
import frc.robot.controls.JoystickOptions;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drivetrain.VisionSubsystem;
import frc.robot.subsystems.drivetrain.swerve.SwerveSubsystem;
import frc.robot.subsystems.drivetrain.swerve.commands.SwerveJoystickDriveCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.neopixel.NeoPixelSubsystem;
import frc.robot.subsystems.shooter.ShooterCalibratorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.Logging;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  private final CommandJoystick driver = new CommandJoystick(JoystickOptions.DualSense);

  private final NeoPixelSubsystem neopixelSubsystem;

  private final SwerveSubsystem swerveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private final IntakeSubsystem intakeSubsystem;
  private final ClimbSubsystem climbSubsystem;

  private final ShooterSubsystem shooterSubsystem;
  private final ShooterCalibratorSubsystem shooterCalibratorSubsystem;

  /*
   * private final ShooterSubsystem shooterSubsystem;
   * private final ShooterCalibratorSubsystem shooterCalibratorSubsystem;
   */

  public RobotContainer() {
    neopixelSubsystem = new NeoPixelSubsystem();
    

    swerveSubsystem = new SwerveSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    shooterCalibratorSubsystem = new ShooterCalibratorSubsystem(shooterSubsystem);
    visionSubsystem = new VisionSubsystem(swerveSubsystem);
    climbSubsystem = new ClimbSubsystem();

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

        
    driver.getPovDown().onTrue(shooterCalibratorSubsystem.IncreaseRPMCommand(-100));
    driver.getPovUp().onTrue(shooterCalibratorSubsystem.IncreaseRPMCommand(100));

    
    // driver.getBtnLeft().onTrue(climbSubsystem.Toggle());

    driver.getR1().onTrue(intakeSubsystem.Toggle());

    driver.getL1().toggleOnTrue(shooterSubsystem.prepareShooterCommand());

    driver.getBtnDown().and(shooterSubsystem.isReadyToShoot()).whileTrue(shooterSubsystem.FeedCommand());

  }

  double time = 0;

  public Command getAutonomousCommand() {
    return Commands.runOnce(() -> time = Timer.getFPGATimestamp()).andThen(new PathPlannerAuto("TESTAUTO"))
        .andThen(() -> {
          System.out.println("AUTO TIME: " + (Timer.getFPGATimestamp() - time));
        }).finallyDo(() -> swerveSubsystem.Stop());
  }

  public void updateNetworkTables() {
    driver.updateNetworkTables();
  }

}
