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
    private TalonSRX talonSRX;
    private CANSparkMax sparkMax;
    private VictorSPX victorSPX;

    private ControllerType type = ControllerType.PWM;

    public AutoDetectMotorController(int id) {
        // Iterate over each motor controller and find the active one

        // Try TalonSRX
        talonSRX = new TalonSRX(id + 1); // All CAN addresses are off by one
        if (talonSRX.getFirmwareVersion() != -1) {
            type = ControllerType.CANTalonSRX;
            System.out.println("Found TalonSRX for ID #" + id  + " with FW Version " + talonSRX.getFirmwareVersion());
            return;
        }

        // Not talon, close and move on
        talonSRX.DestroyObject();
        talonSRX = null;

        // Try Victor SPX
        victorSPX = new VictorSPX(id + 1); // All CAN Addresses are off by one
        if (victorSPX.getFirmwareVersion() != -1) {
            type = ControllerType.CANVictorSPX;
            System.out.println("Found VictorSPX for ID #" + id  + " with FW Version " + victorSPX.getFirmwareVersion() );
            return;
        }

        // Not victor, close and move on
        victorSPX.DestroyObject();
        victorSPX = null;

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
        } else if (type == ControllerType.CANTalonSRX && talonSRX != null) {
            talonSRX.set(ControlMode.PercentOutput, speed);
        } else if (type == ControllerType.CANVictorSPX && victorSPX != null) {
            victorSPX.set(ControlMode.PercentOutput, speed);
        }
    }

    public void setInverted(boolean invert) {
        if (type == ControllerType.PWM && pwm != null) {
            pwm.setInverted(invert);
        } else if (type == ControllerType.CANSparkMax && sparkMax != null) {
            sparkMax.setInverted(invert);
        } else if (type == ControllerType.CANTalonSRX && talonSRX != null) {
            talonSRX.setInverted(invert);
        } else if (type == ControllerType.CANVictorSPX && victorSPX != null) {
            victorSPX.setInverted(invert);
        }
    }
}
