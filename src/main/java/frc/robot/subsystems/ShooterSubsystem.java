package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooterMotor;
    private static final double SHOOTER_SPEED = 0.8; // 80% speed - adjust as needed
    
    public ShooterSubsystem() {
        shooterMotor = new CANSparkMax(13, MotorType.kBrushless);
        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setInverted(true);
        shooterMotor.setSmartCurrentLimit(40); // Adjust current limit as needed
    }
    
    public void startShooter() {
        shooterMotor.set(SHOOTER_SPEED);
    }
    
    public void stopShooter() {
        shooterMotor.set(0);
    }
} 