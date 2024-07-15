// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.AutoDetectMotorController;

public class DriveSubsystem extends SubsystemBase {

  public static enum DriveType {
    PWM,
    CANSparkMax
  }

  private AutoDetectMotorController rearRight, frontRight, rearLeft, frontLeft;

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    rearRight = new AutoDetectMotorController(Constants.RearRight);
    frontRight = new AutoDetectMotorController(Constants.FrontRight);
    rearLeft = new AutoDetectMotorController(Constants.RearLeft);
    frontLeft = new AutoDetectMotorController(Constants.FrontLeft);

    rearRight.setInverted(true);
    frontRight.setInverted(true);
  }

  public void arcadeDrive(double forwardSpeed, double turnSpeed) {
    double left = -forwardSpeed + turnSpeed;
    double right = -forwardSpeed - turnSpeed;
    rearRight.set(right);
    frontRight.set(right);
    rearLeft.set(left);
    frontLeft.set(left);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    rearRight.set(rightSpeed);
    frontRight.set(rightSpeed);
    rearLeft.set(leftSpeed);
    frontLeft.set(leftSpeed);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
