// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
private final ShooterIO io;
private final SysIdRoutine shooterSysIdRoutine =
  new SysIdRoutine(
    new SysIdRoutine.Config(null, null, null) , 
    new SysIdRoutine.Mechanism(null, null, null));        

  // private PhoenixPIDController shooterController = new PhoenixPIDController(0, 0, 0);

  public Shooter(ShooterIO io) {
  this.io = io;
 
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  private void runShooter() {
    io.setVoltsShooter1(0);
    io.setVoltsShooter2(0);
  }

 
  

}