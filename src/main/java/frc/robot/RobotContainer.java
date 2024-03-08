// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ClimbDownCommand;
import frc.robot.commands.ClimbUpCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeClaw;
import frc.robot.commands.MoveToAmpAngle;
import frc.robot.commands.MoveToIntakeAngle;
import frc.robot.commands.MoveToSpeakerAngle;
import frc.robot.commands.MoveToStowAngle;
import frc.robot.commands.OuttakeClaw;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.auto.AutoIntake;
import frc.robot.commands.auto.AutoMoveToIntake;
import frc.robot.commands.auto.AutoMoveToShoot;
import frc.robot.commands.auto.AutoShoot;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SendableChooser<Command> autoSelector;

  private final Swerve swerve;
  private final Pivot pivot;
  private final Intake intake;
  // private final Climb climb;

  // private final ClimbDownCommand climbDownCommand;
  // private final ClimbUpCommand climbUpCommand;
  private final IntakeClaw intakeClaw;
  private final OuttakeClaw outtakeClaw;
  private final MoveToAmpAngle moveToAmpAngle;
  private final MoveToIntakeAngle moveToIntakeAngle;
  private final Command resetPose2d;
  private final MoveToSpeakerAngle moveToSpeakerAngle;
  // private final MoveToStowAngle moveToStowAngle;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve = new Swerve();
    pivot = new Pivot();
    intake = new Intake();
    // climb = new Climb();

    swerve.setDefaultCommand(
      new SwerveDrive(
        swerve, 
        () -> -Constants.Input.SWERVE_X_INPUT.get().getAsDouble(), 
        () -> Constants.Input.SWERVE_Y_INPUT.get().getAsDouble(), 
        () -> -Constants.Input.SWERVE_ROTATION_INPUT.get().getAsDouble(), 
        () -> false)
      );    

    // climbDownCommand = new ClimbDownCommand(climb);
    // climbUpCommand = new ClimbUpCommand(climb);
    intakeClaw = new IntakeClaw(intake);
    outtakeClaw = new OuttakeClaw(intake, pivot);
    moveToAmpAngle = new MoveToAmpAngle(pivot);
    // pivot.setDefaultCommand(moveToAmpAngle);
    moveToIntakeAngle = new MoveToIntakeAngle(pivot);
    moveToSpeakerAngle = new MoveToSpeakerAngle(pivot, swerve);
    // moveToStowAngle = new MoveToStowAngle(pivot);

    resetPose2d = new FunctionalCommand(() -> {}, () -> swerve.resetPoseWithLimelight(), (x) -> {}, () -> false, swerve);

    // Configure the trigger bindings
    configureBindings();
    autoSelector = AutoBuilder.buildAutoChooser();
    configureAuto();
  }

  private void configureAuto() {
    // NamedCommands.registerCommand("AutoMoveToIntake", new AutoMoveToIntake(pivot));
    // NamedCommands.registerCommand("AutoIntake", new AutoIntake(intake));
    // NamedCommands.registerCommand("AutoMoveToShoot", new AutoMoveToShoot(pivot));
    // NamedCommands.registerCommand("AutoShoot", new AutoShoot(intake));

    autoSelector.addOption("BM2", AutoBuilder.buildAuto("BM2"));
    autoSelector.addOption("BM3", AutoBuilder.buildAuto("BM3"));
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
    Constants.Input.testA.get().whileTrue(intakeClaw);
    Constants.Input.testB.get().whileTrue(outtakeClaw);
    Constants.Input.testX.get().whileTrue(moveToSpeakerAngle);
    Constants.Input.testY.get().whileTrue(moveToIntakeAngle);
    
    Constants.Input.lBumper.get().whileTrue(resetPose2d);
    
    // Constants.Input.rTrigger.getButton().castTo(Trigger::new).whileTrue(new ParallelCommandGroup(intakeClaw, moveToIntakeAngle));
    // Constants.Input.rBumper.get().whileTrue(new SequentialCommandGroup(moveToSpeakerAngle, new OuttakeClaw(intake, pivot)));
    // Constants.Input.lBumper.get().whileTrue(new SequentialCommandGroup(moveToAmpAngle, new OuttakeClaw(intake, pivot)));
  }

  public void runSwerveOdometry() {
    swerve.updateOdometry();
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoSelector.getSelected();
  }
}