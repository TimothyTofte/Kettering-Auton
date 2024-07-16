// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.GyroAutos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoTank;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveStraight extends SequentialCommandGroup {
  /** Creates a new DriveStraight. */
  public DriveStraight(DriveSubsystem driveSubsystem, double time) {
    // while (time > timer.get()) {
    //   System.out.println("time: " + timer.get());
    //   System.out.println(time > timer.get());
      addCommands(
        new AutoTank(driveSubsystem, () -> 0.3, () -> 0).withTimeout(15)
      );
    // }
  }
}
