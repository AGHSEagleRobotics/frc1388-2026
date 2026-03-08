// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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

public class HoodIOKraken implements HoodIO {
  
  
  private VelocityTorqueCurrentFOC hoodMotorVelocityRequest;
  
  private VoltageOut hoodMotorVoltageRequest;

  private MotionMagicVoltage hoodMotorPositionRequest;
  
  
  StatusSignal<AngularVelocity> hoodMotorVelocityStatusSignal;
  
  StatusSignal<Voltage> hoodMotorVoltageStatusSignal;
  
  StatusSignal<Current> hoodMotorCurrentAmpsStatusSignal;
  
  StatusSignal<Temperature> hoodMotorTempCelsiusStatusSignal;

  StatusSignal<Angle> hoodMotorPositionStatusSignal;
  
  private final TalonFX hoodMotor;
  private final CANcoder CANcoder;

  private final Follower followRequest = new Follower(38, MotorAlignmentValue.Opposed);
  public HoodIOKraken() {

    hoodMotor = new TalonFX(38);
    CANcoder = new CANcoder(51);

    
    hoodMotorVelocityStatusSignal = hoodMotor.getVelocity();
    
    hoodMotorVoltageStatusSignal = hoodMotor.getMotorVoltage();
    
    hoodMotorCurrentAmpsStatusSignal = hoodMotor.getStatorCurrent();
    
    hoodMotorTempCelsiusStatusSignal = hoodMotor.getDeviceTemp();
    
    hoodMotorPositionStatusSignal = hoodMotor.getPosition();
    
    hoodMotorVoltageRequest = new VoltageOut(0);
    hoodMotorPositionRequest = new MotionMagicVoltage(0);
    
    configureCANcoder(CANcoder);
    configureHoodMotor(hoodMotor);
  }
  
  @Override
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
      hoodMotorVelocityStatusSignal,
      hoodMotorVoltageStatusSignal,
      hoodMotorCurrentAmpsStatusSignal, 
      hoodMotorTempCelsiusStatusSignal,
      hoodMotorPositionStatusSignal);
      
      inputs.hoodMotorVelocityRPS = hoodMotorVelocityStatusSignal.getValueAsDouble();
      inputs.hoodMotorCurrentAmps = hoodMotorCurrentAmpsStatusSignal.getValueAsDouble();
      inputs.hoodMotorTempCelsius = hoodMotorTempCelsiusStatusSignal.getValueAsDouble();
      inputs.hoodMotorVoltage = hoodMotorVoltageStatusSignal.getValueAsDouble();
      inputs.hoodMotorPosition = hoodMotorPositionStatusSignal.getValueAsDouble();
    }

  public void configureHoodMotor(TalonFX hoodMotor) {
    TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();

    hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hoodMotorConfig.CurrentLimits.SupplyCurrentLimit = HoodConstants.SUPPLY_CURRENT_LIMIT_HOOD;
    hoodMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    hoodMotorConfig.Slot0.kA = 0;
    hoodMotorConfig.Slot0.kG = 0;
    hoodMotorConfig.Slot0.kV = 0;
    hoodMotorConfig.Slot0.kP = 0;
    hoodMotorConfig.Slot0.kI = 0;
    hoodMotorConfig.Slot0.kD = 0;
    hoodMotorConfig.Slot0.kS = 0;

    // TODO: CORRECT LATER
    hoodMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hoodMotorConfig.Feedback.FeedbackSensorSource = 
    FeedbackSensorSourceValue.RemoteCANcoder;

    // TODO: CHECK VALUE
    hoodMotorConfig.Feedback.SensorToMechanismRatio = 1.0;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = hoodMotor.getConfigurator().apply(hoodMotorConfig);
      if (status.isOK())
        break;
    }
  }

  public void configureCANcoder(CANcoder CANcoder) {
    CANcoderConfiguration CANcoderConfig = new CANcoderConfiguration();

    //TODO: CHANGE LATER
    CANcoderConfig.MagnetSensor.MagnetOffset = HoodConstants.HOOD_OFFSET;
    // Gives 0.0 to 1.0 range (full rotation, no discontinuity)
    CANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    
    CANcoder.getConfigurator().apply(CANcoderConfig);
  }

  @Override
  public void setPosition(double position){
    hoodMotor.setControl(hoodMotorPositionRequest.withPosition(position));
  }

  @Override
  public void setVoltage(double volts){
    hoodMotor.setControl(hoodMotorVoltageRequest.withOutput(volts));
  }

  public double getPosition() {
    return CANcoder.getAbsolutePosition().getValueAsDouble() * 360.0;

  }
}
