// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Filter;
import frc.robot.Constants;
import frc.robot.utils.AutoDetectMotorController;

public class DriveSubsystem extends SubsystemBase {

  private AHRS gyro;

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

    gyro = new AHRS(SPI.Port.kMXP);

    zeroGyro();
  }

  public void drive(double left, double right) {
    left *= Constants.maxSpeed;
    right *= Constants.maxSpeed;
    SmartDashboard.putNumber("Unfiltered Right", right);
    SmartDashboard.putNumber("Unfiltered Left", left);
    left = Filter.deadband(left, 0.05);
    right = Filter.deadband(right, 0.05);

    double leftSpeed = Filter.cutoffFilter(left+right);

    double rightSpeed = Filter.cutoffFilter(left-right);

    SmartDashboard.putNumber("Commanded Speed Left", leftSpeed);
    SmartDashboard.putNumber("Commanded Speed Right", rightSpeed);

    frontLeft.set(leftSpeed);
    frontRight.set(rightSpeed);

    // driveLeft.set(ControlMode.PercentOutput, leftSpeed * 1.25);
    // driveRight.set(ControlMode.PercentOutput, rightSpeed);
    rearLeft.set(leftSpeed);
    rearRight.set(rightSpeed);

    // followerLeft.set(ControlMode.PercentOutput, leftSpeed);
    // followerRight.set(ControlMode.PercentOutput, rightSpeed);
  }

  public void zeroGyro() {
    gyro.reset();
  }

  public double getGyroYaw() {
    // return gyro.getYaw() < 0 ? gyro.getYaw() + 360 : gyro.getYaw();
    return gyro.getYaw();
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
