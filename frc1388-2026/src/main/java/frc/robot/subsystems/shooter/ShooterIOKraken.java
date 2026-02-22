// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterIOKraken implements ShooterIO {
  //velocitytorquecurrentFOC requests / used for setting velocity
  private VelocityTorqueCurrentFOC shootMotor1VelocityRequest;
  private VelocityTorqueCurrentFOC shootMotor2VelocityRequest;

  //statussignals
  private StatusSignal<AngularVelocity> shootMotor1VelocitySS;
  private StatusSignal<Voltage> shootMotor1VoltageSS;
  private StatusSignal<Current> shootMotor1TorqueCurrentAmpsSS;
  private StatusSignal<Current> shootMotor1SupplyCurrentAmpsSS;
  private StatusSignal<Temperature> shootMotor1TempCelsiusSS;
  
  private StatusSignal<AngularVelocity> shootMotor2VelocitySS;
  private StatusSignal<Voltage> shootMotor2VoltageSS;
  private StatusSignal<Current> shootMotor2TorqueCurrentAmpsSS;
  private StatusSignal<Current> shootMotor2SupplyCurrentAmpsSS;
  private StatusSignal<Temperature> shootMotor2TempCelsiusSS;



  //control
  private final Slot0Configs controllerConfig = new Slot0Configs();
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

  //hardware/talons
  private TalonFX shootMotor1;
  private TalonFX shootMotor2;

  //velocity
  private double shootMotor1Velocity;
  private double shootMotor2Velocity;


  public ShooterIOKraken() {
    shootMotor1 = new TalonFX(shooterConfig.motorID1);
    shootMotor2 = new TalonFX(0);

    // General config
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = shooterConfig.reduction();

    // Controller config;
    controllerConfig.kP = gains.kP();
    controllerConfig.kI = gains.kI();
    controllerConfig.kD = gains.kD();
    controllerConfig.kS = gains.kS();
    controllerConfig.kV = gains.kV();
    controllerConfig.kA = gains.kA();

    // Apply configs
    shootMotor1.getConfigurator().apply(config, 1.0);
    shootMotor2.getConfigurator().apply(config, 1.0);
    shootMotor1.getConfigurator().apply(controllerConfig, 1.0);
    shootMotor2.getConfigurator().apply(controllerConfig, 1.0);

    shootMotor1VelocityRequest = new VelocityTorqueCurrentFOC(0);
    shootMotor2VelocityRequest = new VelocityTorqueCurrentFOC(0);
    
    shootMotor1VelocitySS = shootMotor1.getVelocity();
    shootMotor1VoltageSS = shootMotor1.getMotorVoltage();
    shootMotor1TorqueCurrentAmpsSS = shootMotor1.getTorqueCurrent();
    shootMotor1SupplyCurrentAmpsSS = shootMotor1.getSupplyCurrent();
    shootMotor1TempCelsiusSS = shootMotor1.getDeviceTemp();
    
    shootMotor2VelocitySS = shootMotor2.getVelocity();
    shootMotor2VoltageSS = shootMotor2.getMotorVoltage();
    shootMotor2TorqueCurrentAmpsSS = shootMotor2.getTorqueCurrent();
    shootMotor2SupplyCurrentAmpsSS = shootMotor2.getTorqueCurrent();
    shootMotor2TempCelsiusSS = shootMotor1.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
      100.0, 
      shootMotor1VelocitySS,
      shootMotor1VoltageSS,
      shootMotor1TorqueCurrentAmpsSS,
      shootMotor1SupplyCurrentAmpsSS,
      shootMotor1TempCelsiusSS,
      
      shootMotor2VelocitySS,
      shootMotor2VoltageSS,
      shootMotor2TorqueCurrentAmpsSS,
      shootMotor2SupplyCurrentAmpsSS,
      shootMotor2TempCelsiusSS); 

  }
  @Override
  public void updateInputs(ShooterIOInputs shooterInputs) {
    shooterInputs.shootMotor1Connected = 
      BaseStatusSignal.refreshAll(
        shootMotor1VelocitySS,
        shootMotor1VoltageSS,
        shootMotor1TorqueCurrentAmpsSS,
        shootMotor1SupplyCurrentAmpsSS,
        shootMotor1TempCelsiusSS)
      .isOK();
    shooterInputs.shootMotor2Connected = 
      BaseStatusSignal.refreshAll(
        shootMotor2VelocitySS,
        shootMotor2VoltageSS,
        shootMotor2TorqueCurrentAmpsSS,
        shootMotor2SupplyCurrentAmpsSS,
        shootMotor2TempCelsiusSS)
      .isOK();

    //setting signals / updating motor inputs
    shooterInputs.shootMotor1VelocityRPS = shootMotor1VelocitySS.getValueAsDouble();
    shooterInputs.shootMotor1ReferenceVelocityRPS = this.shootMotor1Velocity;
    shooterInputs.shootMotor1ClosedLoopReferenceRPS = shootMotor1.getClosedLoopReference().getValueAsDouble();
    shooterInputs.shootMotor1Voltage = shootMotor1VoltageSS.getValueAsDouble();
    shooterInputs.shootMotor1TorqueCurrentAmps = shootMotor1TorqueCurrentAmpsSS.getValueAsDouble();
    shooterInputs.shootMotor1SupplyCurrentAmps = shootMotor1SupplyCurrentAmpsSS.getValueAsDouble();
    shooterInputs.shootMotor1TempCelsius = shootMotor1TempCelsiusSS.getValueAsDouble();

    shooterInputs.shootMotor2VelocityRPS = shootMotor2VelocitySS.getValueAsDouble();
    shooterInputs.shootMotor2ReferenceVelocityRPS = this.shootMotor2Velocity;
    shooterInputs.shootMotor2ClosedLoopReferenceRPS = shootMotor2.getClosedLoopReference().getValueAsDouble();
    shooterInputs.shootMotor2Voltage = shootMotor2VoltageSS.getValueAsDouble();
    shooterInputs.shootMotor2TorqueCurrentAmps = shootMotor2TorqueCurrentAmpsSS.getValueAsDouble();
    shooterInputs.shootMotor2SupplyCurrentAmps = shootMotor2SupplyCurrentAmpsSS.getValueAsDouble();
    shooterInputs.shootMotor2TempCelsius = shootMotor2TempCelsiusSS.getValueAsDouble();
      
  }
  @Override
    public void setShooterVelocity(double motor1RPS, double motor2RPS) {
      shootMotor1.setControl(velocityControl.withVelocity(motor1RPS));
      shootMotor2.setControl(velocityControl.withVelocity(motor2RPS));
    }
  @Override
    public void setShooterVolts(double motor1Volts, double motor2Volts) {
      shootMotor1.setControl(voltageControl.withOutput(motor1Volts));
      shootMotor2.setControl(voltageControl.withOutput(motor1Volts));
  }
   @Override
  
  // @Override
  //   public void setVoltsMotor1(double volts) {
  //     shootMotor1.setControl(shootMotor1VoltageRequest.withOutput(volts));
  //   }
  //   public void setVoltsMotor2(double volts) {
  //     shootMotor1.setControl(shootMotor2VoltageRequest.withOutput(volts));
  //   }

  }