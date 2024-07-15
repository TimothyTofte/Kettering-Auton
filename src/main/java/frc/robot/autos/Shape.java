// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shape extends SequentialCommandGroup {
  /** Creates a new Task6. */
  public Shape(DriveSubsystem driveSubsystem, int sides, boolean curve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // 0.506
    addCommands(
      new TankDrive(driveSubsystem, () -> 0.5, () -> 0).withTimeout(0.5)
    );
    // Setting the value to 1 outputs the correct number of sides because of how this is written
    if (curve) {
      for (int i = 1; i < sides; i++) {
        driveSide(driveSubsystem, sides, 0.15);
      }
    } else {
      for (int i = 1; i < sides; i++) {
        driveSide(driveSubsystem, sides);
      }
    }

    addCommands(
      new TankDrive(driveSubsystem, () -> 0, () -> 0)
    );

  }

  private void driveSide(DriveSubsystem driveSubsystem, int sides) {
    double time = 3.04 / sides;
    addCommands(
      new TankDrive(driveSubsystem, () -> 0, () -> 0.5).withTimeout(time),
      new TankDrive(driveSubsystem, () -> 0.5, () -> 0).withTimeout(0.3)
    );
  }

  private void driveSide(DriveSubsystem driveSubsystem, int sides, double curve) {
    double turnPower = 0.3;
    double time = ((1-turnPower) * 6.10) / sides;
    addCommands(
      new TankDrive(driveSubsystem, () -> curve, () -> turnPower).withTimeout(time),
      new TankDrive(driveSubsystem, () -> 0.5, () -> 0).withTimeout(0.5)
    );
  }
}
