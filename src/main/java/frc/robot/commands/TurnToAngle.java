// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngle extends Command {
  DriveSubsystem driveSubsystem;
  DoubleSupplier angle;
  DoubleSupplier speed = () -> 0;
  /** Creates a new TurnToAngle. */
  public TurnToAngle(DriveSubsystem driveSubsystem, DoubleSupplier angle, DoubleSupplier speed) {
    this.driveSubsystem = driveSubsystem;
    this.angle = angle.getAsDouble() < 0 ? () -> angle.getAsDouble() + 360 : angle;
    this.speed = speed;
    addRequirements(driveSubsystem);
  }
  
  public TurnToAngle(DriveSubsystem driveSubsystem, DoubleSupplier angle) {
    this.driveSubsystem = driveSubsystem;
    this.angle = angle;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Commanded Angle", angle.getAsDouble());
    driveSubsystem.drive(speed.getAsDouble(), -(driveSubsystem.getGyroYaw() - angle.getAsDouble()) / 10);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
