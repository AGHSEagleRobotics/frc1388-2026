// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

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
  private VelocityTorqueCurrentFOC kickerMotorVelocityRequest;
  private VelocityTorqueCurrentFOC hoodEncoderVelocityRequest;
  private VoltageOut shootMotor1VoltageRequest;
  private VoltageOut hoodEncoderVoltageRequest;
  private VoltageOut kickerMotorVoltageRequest;
  private VoltageOut shootMotor2VoltageRequest;

  //used to get shootmotor velocity values
  StatusSignal<AngularVelocity> shootMotor1VelocityStatusSignal;
  StatusSignal<AngularVelocity> shootMotor2VelocityStatusSignal;
  StatusSignal<AngularVelocity> kickerMotorVelocityStatusSignal;
  StatusSignal<AngularVelocity> hoodEncoderVelocityStatusSignal;

  //used to get voltage values
  StatusSignal<Voltage> shootMotor1VoltageStatusSignal;
  StatusSignal<Voltage> shootMotor2VoltageStatusSignal;
  StatusSignal<Voltage> kickerMotorVoltageStatusSignal;
  StatusSignal<Angle> hoodEncoderVoltageStatusSignal;

  //used to get currentamps
  StatusSignal<Current> shootMotor1CurrentAmpsStatusSignal;
  StatusSignal<Current> shootMotor2CurrentAmpsStatusSignal;
  StatusSignal<Current> kickerMotorCurrentAmpsStatusSignal;
  StatusSignal<Current> hoodEncoderCurrentAmpsStatusSignal;

  //used to get motor temp
  StatusSignal<Temperature> shootMotor1TempCelsiusStatusSignal;
  StatusSignal<Temperature> shootMotor2TempCelsiusStatusSignal;
  StatusSignal<Temperature> kickerMotorTempCelsiusStatusSignal;
  StatusSignal<Temperature> hoodEncoderTempCelsiusStatusSignal;

  //krakenx60s
  private TalonFX shootMotor1;
  private TalonFX shootMotor2;
  private TalonFX kickerMotor;

  //Encoder
  private CANcoder hoodEncoder;

  //velocity
  private double shootMotor1Velocity;
  private double shootMotor2Velocity;
  private double kickerMotorVelocity;
  private double hoodEncoderVelocity;



  public ShooterIOKraken() {
    shootMotor1 = new TalonFX(0);
    shootMotor2 = new TalonFX(0);
    kickerMotor = new TalonFX(0);
    hoodEncoder = new CANcoder(0);

    shootMotor1VelocityRequest = new VelocityTorqueCurrentFOC(0);
    shootMotor2VelocityRequest = new VelocityTorqueCurrentFOC(0);
    kickerMotorVelocityRequest = new VelocityTorqueCurrentFOC(0);
    hoodEncoderVelocityRequest = new VelocityTorqueCurrentFOC(0);
    
    shootMotor1VelocityStatusSignal = shootMotor1.getVelocity();
    shootMotor2VelocityStatusSignal = shootMotor2.getVelocity();
    kickerMotorVelocityStatusSignal = kickerMotor.getVelocity();
    hoodEncoderVelocityStatusSignal = hoodEncoder.getVelocity();

    shootMotor1VoltageStatusSignal = shootMotor1.getMotorVoltage();
    shootMotor2VoltageStatusSignal = shootMotor2.getMotorVoltage();
    kickerMotorVoltageStatusSignal = kickerMotor.getMotorVoltage();
    hoodEncoderVoltageStatusSignal = hoodEncoder.getAbsolutePosition();

    shootMotor1CurrentAmpsStatusSignal = shootMotor1.getStatorCurrent();
    shootMotor2CurrentAmpsStatusSignal = shootMotor2.getStatorCurrent();
    kickerMotorCurrentAmpsStatusSignal = kickerMotor.getStatorCurrent();

    shootMotor1TempCelsiusStatusSignal = shootMotor1.getDeviceTemp();
    shootMotor2TempCelsiusStatusSignal = shootMotor2.getDeviceTemp();
    kickerMotorTempCelsiusStatusSignal = kickerMotor.getDeviceTemp();
  
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
      shootMotor2.setControl(shootMotor2VelocityRequest.withVelocity(rps));
    }

    public void setVelocityKickerMotor(double rps) {
      kickerMotor.setControl(kickerMotorVelocityRequest.withVelocity(rps));
    }

    public void setVoltsMotor1(double volts) {
      shootMotor1.setControl(shootMotor1VoltageRequest.withOutput(volts));
    }
    public void setVoltsMotor2(double volts) {
      shootMotor2.setControl(shootMotor2VoltageRequest.withOutput(volts));
    }

    public void setVoltskickerMotor(double volts) {
      kickerMotor.setControl(kickerMotorVoltageRequest.withOutput(volts));
    }

    public void setEncoderAngle(double rad) {
      hoodEncoder.setPosition(rad);
    }



}