package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmFeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SimpleAuto extends SequentialCommandGroup {
    public SimpleAuto(DriveSubsystem driveSubsystem, ArmFeedSubsystem armFeed, ShooterSubsystem shooter) {
        addCommands(
            // Create a parallel command group that runs the shooter alongside everything else
            new ParallelCommandGroup(
                new RunShooter(shooter),
                new SequentialCommandGroup(
                    // Start with arm up
                    new SetArmPosition(armFeed, true),
                    
                    // Drive forward for 5 seconds
                    new AutoTank(driveSubsystem, () -> 0.5, () -> 0.5)
                        .withTimeout(5.0),
                    
                    // Turn right for 1 second
                    new AutoTank(driveSubsystem, () -> 0.5, () -> -0.5)
                        .withTimeout(1.0),
                    
                    // Drive forward for 3 seconds
                    new AutoTank(driveSubsystem, () -> 0.5, () -> 0.5)
                        .withTimeout(3.0),
                    
                    // Finally, set arm down
                    new SetArmPosition(armFeed, false)
                )
            )
        );
    }
} 