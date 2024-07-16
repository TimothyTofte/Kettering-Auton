// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngle extends CommandBase {
  DriveSubsystem driveSubsystem;
  DoubleSupplier angle;
  /** Creates a new TurnToAngle. */
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
    driveSubsystem.drive(0, -(driveSubsystem.getGyroYaw() - angle.getAsDouble()) / 45);
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
