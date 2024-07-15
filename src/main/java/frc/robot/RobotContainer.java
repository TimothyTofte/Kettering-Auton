// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autos.Shape;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  // Joystick
  private final Joystick m_stick0 = new Joystick(0);


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
    m_driveSubsystem.setDefaultCommand(new TankDrive(
      m_driveSubsystem,
      () -> m_stick0.getRawAxis(XboxController.Axis.kLeftY.value),
      () -> Filter.powerCurve(m_stick0.getRawAxis(XboxController.Axis.kRightX.value), 3)
    ));
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

  }

  public void diagnostics() {
    SmartDashboard.putNumber("LeftY", m_stick0.getRawAxis(XboxController.Axis.kLeftY.value));
    SmartDashboard.putNumber("RightY", m_stick0.getRawAxis(XboxController.Axis.kRightX.value));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new TestAuto(m_driveSubsystem);
    // return new Task2(m_driveSubsystem);
    // return new Task3(m_driveSubsystem);
    // return new Task4(m_driveSubsystem);
    // return new Task5(m_driveSubsystem);
    return new Shape(m_driveSubsystem, 6, true);
  }
}
