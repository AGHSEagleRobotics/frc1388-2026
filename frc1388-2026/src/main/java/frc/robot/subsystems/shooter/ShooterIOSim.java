// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterIOSim implements ShooterIO {
  private TalonFXSimState shootMotorSim;
  private EncoderSim shootEncoderSim;

  private NetworkTableEntry shootFlywheelMassKg;

  /** Creates a new ShooterIOSim. */
  public ShooterIOSim(TalonFXSimState shootMotor1Sim, EncoderSim shootEncoderSim) {
    this.shootMotorSim = shootMotorSim;
    this.shootEncoderSim = shootEncoderSim;

  }
  
  
}
