package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmFeedSubsystem;

public class SetArmPosition extends Command {
    private final ArmFeedSubsystem armFeed;
    private final boolean isUp;
    
    public SetArmPosition(ArmFeedSubsystem armFeed, boolean isUp) {
        this.armFeed = armFeed;
        this.isUp = isUp;
        addRequirements(armFeed);
    }
    
    @Override
    public void initialize() {
        if (isUp) {
            armFeed.setUp();
        } else {
            armFeed.setDown();
        }
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
} 