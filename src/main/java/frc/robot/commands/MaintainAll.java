// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVision;

public class MaintainAll extends Command {
  PhotonVision photonVision;
  DriveSubsystem driveSubsystem;

  // PID for the forwardSpeed PID loop
  final double P_GAIN = 0.75;
  final double D_GAIN = 0;
  PIDController controller = new PIDController(P_GAIN, 0, D_GAIN);

  // PID for the rotSpeed PID loop
  final double ANGULAR_P = 0.2;

  final double ANGULAR_D = 0.0;

  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  // Initialize the variable for speed, not really needed here but why not
  private double forwardSpeed;
  private double rotSpeed;

  /** Creates a new MaintainAll. */
  public MaintainAll(PhotonVision photonVision, DriveSubsystem driveSubsystem) {
    this.photonVision = photonVision;
    this.driveSubsystem = driveSubsystem;
    addRequirements(photonVision, driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yaw = photonVision.getYaw();
    double range = photonVision.getDistance();

    /**
    The PID controller didn't like calculating when the value was zero since I return zero when no tag is found
    so these ternary operators will just set the speed to zero when there isn't an april tag detected
    */
    rotSpeed = yaw != 0 ? -turnController.calculate(yaw, 0) / 10 : 0;
    
    forwardSpeed = range != 0 ? controller.calculate(range, 1.5) : 0;
    
    // Drives the robot at the desired forward and rotational speed
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
