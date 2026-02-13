// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterIOKraken implements ShooterIO {
  /** Creates a new ShooterIOKraken. */

  private TalonFX shootMotor1;
  private TalonFX shootMotor2;

  
  public ShooterIOKraken() {
    shootMotor1 = new TalonFX(0);
    shootMotor2 = new TalonFX(0);
    
    //configShootMotor(shootMotor1, false, false);
    //configShootMotor(shootMotor2, false , false);


  }
}
