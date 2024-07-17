// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.constant.DirectMethodHandleDesc;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autos.GoToDistance;
import frc.robot.autos.GoToTagAngle;
import frc.robot.autos.Shape;
import frc.robot.autos.Task2;
import frc.robot.autos.GyroAutos.DriveStraight;
import frc.robot.autos.GyroAutos.SmartShape;
import frc.robot.commands.DriveToTag;
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
        () -> -controller0.getRawAxis(XboxController.Axis.kLeftY.value),
        () -> Filter.powerCurve(controller0.getRawAxis(XboxController.Axis.kRightX.value), 3)));
    // driveSubsystem.setDefaultCommand(new TurnToAngle(driveSubsystem, () -> Rotation2d.fromRadians(Math.atan2(Filter.deadband(controller0.getRawAxis(XboxController.Axis.kRightY.value), 0.1), Filter.deadband(controller0.getRawAxis(XboxController.Axis.kRightX.value), 0.1))).getDegrees(), () -> controller0.getRawAxis(XboxController.Axis.kLeftY.value)));
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
    // aButton.whileTrue(new TurnToAngle(driveSubsystem, () -> photonVision.getYaw()));
    aButton.whileTrue(new DriveToTag(driveSubsystem, photonVision));
  }

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
    // return new TestAuto(m_driveSubsystem);
    // return new Task2(driveSubsystem);
    // return new Task3(m_driveSubsystem);
    // return new Task4(m_driveSubsystem);
    // return new Task5(m_driveSubsystem);
    // return new Shape(driveSubsystem, 6, false);
    // return new SmartShape(driveSubsystem, 4);
    // return new GoToDistance(driveSubsystem, photonVision);
    return new DriveToTag(driveSubsystem, photonVision);
    // return new DriveStraight(driveSubsystem, 5);
  }
}
