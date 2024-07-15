// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.GyroAutos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SmartShape extends SequentialCommandGroup {
  /** Creates a new SmartShape. */
  public SmartShape(DriveSubsystem driveSubsystem, int sides) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new TankDrive(driveSubsystem, () -> 0, () -> 0));

    double angle = 360 / sides;

    double offset = driveSubsystem.getGyroYaw();

    for (int i = 0; i < sides; i++) {
      addCommands(new TankDrive(driveSubsystem, () -> 0.5, () -> 0));
      if (Math.abs(driveSubsystem.getGyroYaw() - offset) >= angle) {
        continue;
      } else {
        addCommands(
          new TankDrive(driveSubsystem, () -> 0, () -> 0.5)
        );
      }
    }
  }
}
