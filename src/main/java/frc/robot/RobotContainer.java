// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.MaintainAll;
import frc.robot.commands.MaintainDistance;
import frc.robot.commands.MaintainYaw;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.lib.math.Filter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final PhotonVision photonVision = new PhotonVision();

  // Joystick
  private final Joystick controller0 = new Joystick(0);
  private final JoystickButton yButton = new JoystickButton(controller0, XboxController.Button.kY.value);
  private final JoystickButton aButton = new JoystickButton(controller0, XboxController.Button.kA.value);
  private final JoystickButton bButton = new JoystickButton(controller0, XboxController.Button.kB.value);
  private final JoystickButton xButton = new JoystickButton(controller0, XboxController.Button.kX.value);
  private final POVButton dpadUp = new POVButton(controller0, 0);
  private final POVButton dpadDown = new POVButton(controller0, 180);
  private final POVButton dpadLeft = new POVButton(controller0, 90);
  private final POVButton dpadRight = new POVButton(controller0, 270);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set Default Commands
    setDefaultCommands();
  }

  // Set Subsystem Default Commands
  public void setDefaultCommands() {
    driveSubsystem.setDefaultCommand(new TankDrive(
        driveSubsystem,
        () -> controller0.getRawAxis(XboxController.Axis.kLeftY.value),
        () -> Filter.powerCurve(controller0.getRawAxis(XboxController.Axis.kRightX.value), 3)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    yButton.onTrue(new InstantCommand(() -> driveSubsystem.zeroGyro()));
    dpadUp.whileTrue(new TurnToAngle(driveSubsystem, () -> 0));
    dpadDown.whileTrue(new TurnToAngle(driveSubsystem, () -> 180));
    dpadLeft.whileTrue(new TurnToAngle(driveSubsystem, () -> 90));
    dpadRight.whileTrue(new TurnToAngle(driveSubsystem, () -> -90));
    bButton.whileTrue(new MaintainDistance(photonVision, driveSubsystem));
    aButton.whileTrue(new MaintainYaw(photonVision, driveSubsystem));
    // This basically runs the a & b command
    xButton.whileTrue(new MaintainAll(photonVision, driveSubsystem));
  }

  // I have this set to run during robot periodic which is helpful for debugging
  public void diagnostics() {
    SmartDashboard.putNumber("LeftY", controller0.getRawAxis(XboxController.Axis.kLeftY.value));
    SmartDashboard.putNumber("RightY", controller0.getRawAxis(XboxController.Axis.kRightX.value));
    SmartDashboard.putNumber("Gyro", driveSubsystem.getGyroYaw());
    SmartDashboard.putBoolean("Photon Vision", photonVision.hasTarget());
    SmartDashboard.putNumber("Best April Tag ID", photonVision.getBestTargetID());
    SmartDashboard.putNumber("April Tag Yaw", photonVision.getYaw());
    SmartDashboard.putNumber("Distance to Tag", photonVision.getDistance());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    driveSubsystem.zeroGyro();
    return new MaintainAll(photonVision, driveSubsystem);
  }
}
