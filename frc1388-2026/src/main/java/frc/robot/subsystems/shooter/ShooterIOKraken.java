// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterIOKraken implements ShooterIO {
  /** Creates a new ShooterIOKraken. */
  private TalonFX shootMotor1;
  private TalonFX shootMotor2;
  private CANcoder angleEncoder;
  

public void setVoltsMotor1() {
  shootMotor1.setVoltage(0);
}
public void setVoltsMotor2() {
  shootMotor2.setVoltage(0);
}

public void getShooterRPM() {

}

  public ShooterIOKraken() {
    shootMotor1 = new TalonFX(0);
    shootMotor2 = new TalonFX(0);

    angleEncoder = new CANcoder(0);

    
    
    //configShootMotor(shootMotor1, false, false);
    //configShootMotor(shootMotor2, false , false);
//remember to add throughbore encider

  }
}
