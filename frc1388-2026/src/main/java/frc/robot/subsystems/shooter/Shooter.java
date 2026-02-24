// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.lang.System.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase {

private final ShooterIO io;
private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

private final SysIdRoutine shooterSysIdRoutine =
  new SysIdRoutine(
    new SysIdRoutine.Config(null, null, null) , 
    new SysIdRoutine.Mechanism(null, null, null));        

  public Shooter(ShooterIO io) {
  this.io = io;
 
    
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("shooter subsystem", inputs);

  

  }

 
  

}