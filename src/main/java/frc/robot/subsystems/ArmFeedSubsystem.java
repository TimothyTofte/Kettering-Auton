package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmFeedSubsystem extends SubsystemBase {
    private final Servo armServo;
    
    // Constants for arm positions
    private static final double UP_POSITION = 0.0;    // 0 degrees
    private static final double DOWN_POSITION = 80.0; // 80 degrees
    
    public ArmFeedSubsystem() {
        // Initialize servo on PWM port 0 (adjust port as needed)
        armServo = new Servo(0);
    }
    
    public void setUp() {
        armServo.setAngle(UP_POSITION);
    }
    
    public void setDown() {
        armServo.setAngle(DOWN_POSITION);
    }
    
    public double getPosition() {
        return armServo.getAngle();
    }
} 