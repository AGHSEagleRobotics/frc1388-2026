// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodIOTalonFX implements HoodIO {
  
  
  private VelocityTorqueCurrentFOC hoodMotorVelocityRequest;
  
  private VoltageOut hoodMotorVoltageRequest;
  
  
  StatusSignal<AngularVelocity> hoodMotorVelocityStatusSignal;
  
  StatusSignal<Voltage> hoodMotorVoltageStatusSignal;
  
  StatusSignal<Current> hoodMotorCurrentAmpsStatusSignal;
  
  StatusSignal<Temperature> hoodMotorTempCelsiusStatusSignal;
  
  private final TalonFX hoodMotor;

  private double hoodMotorVelocity;

  public HoodIOTalonFX() {

    hoodMotor = new TalonFX(0);

    hoodMotorVelocityRequest = new VelocityTorqueCurrentFOC(0);

    hoodMotorVelocityStatusSignal = hoodMotor.getVelocity();

    hoodMotorVoltageStatusSignal = hoodMotor.getMotorVoltage();

    hoodMotorCurrentAmpsStatusSignal = hoodMotor.getStatorCurrent();

    hoodMotorTempCelsiusStatusSignal = hoodMotor.getDeviceTemp();

  }

  @Override
  public void updateInputs(HoodIOInputs HoodIOInputs) {
    BaseStatusSignal.refreshAll(
      hoodMotorVelocityStatusSignal,

      hoodMotorVoltageStatusSignal,

      hoodMotorCurrentAmpsStatusSignal, 

      hoodMotorTempCelsiusStatusSignal);
  }

  public void setVelocityHoodMotor(double rps){
    hoodMotor.setControl(hoodMotorVelocityRequest.withVelocity(rps));
  }

  public void setVoltsHoodMotor(double volts){
    hoodMotor.setControl(hoodMotorVoltageRequest.withOutput(volts));
  }
}
