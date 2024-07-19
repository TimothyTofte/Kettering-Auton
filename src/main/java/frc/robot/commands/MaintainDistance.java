// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVision;

public class MaintainDistance extends Command {
  PhotonVision photonVision;
  DriveSubsystem driveSubsystem;
  double forwardSpeed;
  double rotSpeed;

  // PID for the forward speed loop
  final double P_GAIN = 0.5;
  final double D_GAIN = 0;
  PIDController controller = new PIDController(P_GAIN, 0, D_GAIN);

  /** Creates a new MaintainDistance. */
  public MaintainDistance(PhotonVision photonVision, DriveSubsystem driveSubsystem) {
    this.photonVision = photonVision;
    this.driveSubsystem = driveSubsystem;
    addRequirements(photonVision, driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Gets the range
    double range = photonVision.getDistance();
    
    forwardSpeed = range!= 0 ? controller.calculate(range, 1.5) : 0;
    rotSpeed = 0;
    driveSubsystem.drive(forwardSpeed, rotSpeed);
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
