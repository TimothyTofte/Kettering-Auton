// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVision;

public class MaintainYaw extends Command {
  PhotonVision photonVision;
  DriveSubsystem driveSubsystem;

  // PID Loop for the yaw, wahoo
  final double ANGULAR_P = 0.2;

  final double ANGULAR_D = 0.0;

  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  private double forwardSpeed = 0;
  private double rotSpeed = 0;

  /** Creates a new MaintainYaw. */
  public MaintainYaw(PhotonVision photonVision, DriveSubsystem driveSubsystem)
  {
    this.photonVision = photonVision;
    this.driveSubsystem = driveSubsystem;
    addRequirements(photonVision, driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yaw = photonVision.getYaw();

    // If the returned yaw is zero, set the speed to zero, it doesn't like 0/10 :(, not wahoo
    rotSpeed = yaw != 0 ? -turnController.calculate(yaw, 0) / 10 : 0;

    driveSubsystem.drive(forwardSpeed, rotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
