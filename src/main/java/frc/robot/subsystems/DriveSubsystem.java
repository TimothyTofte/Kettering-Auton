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

  public void drive(double speed, double rotation) {
    speed *= Constants.maxSpeed;
    rotation *= Constants.maxSpeed;
    SmartDashboard.putNumber("Unfiltered Right", rotation);
    SmartDashboard.putNumber("Unfiltered Left", speed);
    speed = Filter.deadband(speed, 0.05);
    rotation = Filter.deadband(rotation, 0.05);

    // This doesn't allow the values to exceed 1 or -1
    speed = Filter.cutoffFilter(speed+rotation);

    rotation = Filter.cutoffFilter(speed-rotation);

    // This is just some output stuff for my sanity
    SmartDashboard.putNumber("Commanded Speed Left", speed);
    SmartDashboard.putNumber("Commanded Speed Right", rotation);

    frontLeft.set(speed);
    frontRight.set(rotation);

    rearLeft.set(speed);
    rearRight.set(rotation);
  }

  public void zeroGyro() {
    gyro.reset();
  }

  public double getGyroYaw() {
    return gyro.getYaw();
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
