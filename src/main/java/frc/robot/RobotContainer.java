// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.auto.AutoIntake;
import frc.robot.commands.auto.AutoMoveToIntake;
import frc.robot.commands.auto.AutoMoveToShoot;
import frc.robot.commands.auto.AutoShoot;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Swerve swerve;
  private final Pivot pivot;
  private final Intake intake;
  private final SendableChooser<Command> autoSelector;

  private final SwerveDrive swerveDrive;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve = new Swerve();
    pivot = new Pivot();
    intake = new Intake();

    swerveDrive = new SwerveDrive(swerve, null, null, null, null); // TODO: FILL WITH REAL VALUES

    swerve.setDefaultCommand(swerveDrive);

    // Configure the trigger bindings
    configureBindings();
    autoSelector = AutoBuilder.buildAutoChooser();
    configureAuto();
  }

  private void configureAuto() {
    NamedCommands.registerCommand("AutoMoveToIntake", new AutoMoveToIntake(pivot));
    NamedCommands.registerCommand("AutoIntake", new AutoIntake(intake));
    NamedCommands.registerCommand("AutoMoveToShoot", new AutoMoveToShoot(pivot));
    NamedCommands.registerCommand("AutoShoot", new AutoShoot(intake));

    autoSelector.addOption("BM2", AutoBuilder.followPath(PathPlannerPath.fromPathFile("BM2")));
    autoSelector.addOption("BM3", AutoBuilder.followPath(PathPlannerPath.fromPathFile("BM3")));
    SmartDashboard.putData("Auto Selector", autoSelector);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    

    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoSelector.getSelected();
  }
}
