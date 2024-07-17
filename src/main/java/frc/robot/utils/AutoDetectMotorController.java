// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

/** Add your docs here. */
public class AutoDetectMotorController {

    enum ControllerType {
        PWM,
        CANTalonSRX,
        CANVictorSPX,
        CANSparkMax
    }

    private PWMVictorSPX pwm;
    private TalonSRX talonSrx;
    private CANSparkMax sparkMax;
    private VictorSPX victorSpx;

    private ControllerType type = ControllerType.PWM;

    public AutoDetectMotorController(int id) {
        // Iterate over each motor controller and find the active one

        // Try TalonSRX
        talonSrx = new TalonSRX(id + 1); // All CAN addresses are off by one
        if (talonSrx.getFirmwareVersion() != -1) {
            type = ControllerType.CANTalonSRX;
            System.out.println("Found TalonSRX for ID #" + id  + " with FW Version " + talonSrx.getFirmwareVersion());
            return;
        }

        // Not talon, close and move on
        talonSrx.DestroyObject();
        talonSrx = null;

        // Try Victor SPX
        victorSpx = new VictorSPX(id + 1); // All CAN Addresses are off by one
        if (victorSpx.getFirmwareVersion() != -1) {
            type = ControllerType.CANVictorSPX;
            System.out.println("Found VictorSPX for ID #" + id  + " with FW Version " + victorSpx.getFirmwareVersion() );
            return;
        }

        // Not victor, close and move on
        victorSpx.DestroyObject();
        victorSpx = null;

        // Try Spark Max
        sparkMax = new CANSparkMax(id + 1, MotorType.kBrushed); // All CAN Addresses are off by one
        if (sparkMax.getFirmwareVersion() != 0) {
            type = ControllerType.CANSparkMax;
            System.out.println("Found SparkMax for ID #" + id  + " with FW Version " + sparkMax.getFirmwareVersion() );
            return;
        }

        // Not victor, close and move on
        sparkMax.close();
        sparkMax = null;

        // None of the above devices, assume PWM
        pwm = new PWMVictorSPX(id);
        type = ControllerType.PWM;
        System.out.println("No CAN Devices found for ID #" + id  + ". Defaulting to PWM");
    }

    public void set(double speed) {
        if (type == ControllerType.PWM && pwm != null) {
            pwm.set(speed);
        } else if (type == ControllerType.CANSparkMax && sparkMax != null) {
            sparkMax.set(speed);
        } else if (type == ControllerType.CANTalonSRX && talonSrx != null) {
            talonSrx.set(ControlMode.PercentOutput, speed);
        } else if (type == ControllerType.CANVictorSPX && victorSpx != null) {
            victorSpx.set(ControlMode.PercentOutput, speed);
        }
    }

    public void setInverted(boolean invert) {
        if (type == ControllerType.PWM && pwm != null) {
            pwm.setInverted(invert);
        } else if (type == ControllerType.CANSparkMax && sparkMax != null) {
            sparkMax.setInverted(invert);
        } else if (type == ControllerType.CANTalonSRX && talonSrx != null) {
            talonSrx.setInverted(invert);
        } else if (type == ControllerType.CANVictorSPX && victorSpx != null) {
            victorSpx.setInverted(invert);
        }
    }
}
