// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterIOKraken implements ShooterIO {
  /** Creates a new ShooterIOKraken. */

  //velocitytorquecurrentFOC requests / used for setting velocity
  private VelocityTorqueCurrentFOC shootMotor1VelocityRequest;
  private VelocityTorqueCurrentFOC shootMotor2VelocityRequest;
  private VoltageOut shootMotor1VoltageRequest;
  private VoltageOut shootMotor2VoltageRequest;

  //used to get shootmotor velocity values
  StatusSignal<AngularVelocity> shootMotor1VelocityStatusSignal;
  StatusSignal<AngularVelocity> shootMotor2VelocityStatusSignal;

  //used to get voltage values
  StatusSignal<Voltage> shootMotor1VoltageStatusSignal;
  StatusSignal<Voltage> shootMotor2VoltageStatusSignal;

  //used to get currentamps
  StatusSignal<Current> shootMotor1CurrentAmpsStatusSignal;
  StatusSignal<Current> shootMotor2CurrentAmpsStatusSignal;

  //used to get motor temp
  StatusSignal<Temperature> shootMotor1TempCelsiusStatusSignal;
  StatusSignal<Temperature> shootMotor2TempCelsiusStatusSignal;

  //krakenx60s
  private TalonFX shootMotor1;
  private TalonFX shootMotor2;

  //velocity
  private double shootMotor1Velocity;
  private double shootMotor2Velocity;



  public ShooterIOKraken() {
    shootMotor1 = new TalonFX(0);
    shootMotor2 = new TalonFX(0);

    shootMotor1VelocityRequest = new VelocityTorqueCurrentFOC(0);
    shootMotor2VelocityRequest = new VelocityTorqueCurrentFOC(0);
    
    shootMotor1VelocityStatusSignal = shootMotor1.getVelocity();
    shootMotor2VelocityStatusSignal = shootMotor2.getVelocity();

    shootMotor1VoltageStatusSignal = shootMotor1.getMotorVoltage();
    shootMotor1VoltageStatusSignal = shootMotor2.getMotorVoltage();

    shootMotor1CurrentAmpsStatusSignal = shootMotor1.getStatorCurrent();
    shootMotor2CurrentAmpsStatusSignal = shootMotor2.getStatorCurrent();

    shootMotor1TempCelsiusStatusSignal = shootMotor1.getDeviceTemp();
    shootMotor2TempCelsiusStatusSignal = shootMotor1.getDeviceTemp();
  
  }
  @Override
  public void updateInputs(ShooterIOInputs shooterIOInputs) {
    BaseStatusSignal.refreshAll(
      shootMotor1VelocityStatusSignal,
      shootMotor2VelocityStatusSignal,

      shootMotor1VoltageStatusSignal,
      shootMotor2VoltageStatusSignal,

      shootMotor1CurrentAmpsStatusSignal,
      shootMotor2CurrentAmpsStatusSignal,

      shootMotor1TempCelsiusStatusSignal,
      shootMotor2TempCelsiusStatusSignal);
      

  }

    public void setVelocityMotor1(double rps) {
      shootMotor1.setControl(shootMotor1VelocityRequest.withVelocity(rps));
    }

    public void setVelocityMotor2(double rps) {
      shootMotor1.setControl(shootMotor2VelocityRequest.withVelocity(rps));
    }

    public void setVoltsMotor1(double volts) {
      shootMotor1.setControl(shootMotor1VoltageRequest.withOutput(volts));
    }
    public void setVoltsMotor2(double volts) {
      shootMotor1.setControl(shootMotor2VoltageRequest.withOutput(volts));
    }

}