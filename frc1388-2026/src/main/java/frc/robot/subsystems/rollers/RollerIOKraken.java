// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.RollerConstants;

public class RollerIOKraken implements RollerIO {
  private TalonFX m_bottomRollerMotor;
  private TalonFX m_topRollerMotor;

  private VoltageOut bottomRollerVoltageSet;
  private VoltageOut topRollerVoltageSet;

  private StatusSignal<AngularVelocity> bottomrollerVelocityStatusSignal;
  private StatusSignal<AngularVelocity> toprollerVelocityStatusSignal;

  private StatusSignal<Current> bottomrollerTorqueCurrentStatusSignal;
  private StatusSignal<Current> toprollerTorqueCurrentStatusSignal;

  private StatusSignal<Current> bottomrollerSupplyCurrentStatusSignal;
  private StatusSignal<Current> toprollerSupplyCurrentStatusSignal;

  private StatusSignal<Double> bottomrollerReferenceVelocityStatusSignal;
  private StatusSignal<Double> toprollerReferenceVelocityStatusSignal;

  private StatusSignal<Temperature> bottomrollerTemperatureStatusSignal;
  private StatusSignal<Temperature> toprollerTemperatureStatusSignal;

  private StatusSignal<Voltage> bottomrollerVoltageStatusSignal;
  private StatusSignal<Voltage> toprollerVoltageStatusSignal;

  public RollerIOKraken() {
    m_bottomRollerMotor = new TalonFX(40);
    m_topRollerMotor = new TalonFX(41);

    configurebottomRollerMotor(m_bottomRollerMotor);
    configureTopRoller(m_topRollerMotor);

    bottomRollerVoltageSet = new VoltageOut(0).withUpdateFreqHz(50);
    topRollerVoltageSet = new VoltageOut(0).withUpdateFreqHz(50);

    bottomrollerVelocityStatusSignal = m_bottomRollerMotor.getVelocity();
    toprollerVelocityStatusSignal = m_topRollerMotor.getVelocity();

    bottomrollerTorqueCurrentStatusSignal = m_bottomRollerMotor.getTorqueCurrent();
    toprollerTorqueCurrentStatusSignal = m_topRollerMotor.getTorqueCurrent();

    bottomrollerSupplyCurrentStatusSignal = m_bottomRollerMotor.getSupplyCurrent();
    toprollerSupplyCurrentStatusSignal = m_topRollerMotor.getSupplyCurrent();

    bottomrollerReferenceVelocityStatusSignal = m_bottomRollerMotor.getClosedLoopReference();
    toprollerReferenceVelocityStatusSignal = m_topRollerMotor.getClosedLoopReference();

    bottomrollerTemperatureStatusSignal = m_bottomRollerMotor.getDeviceTemp();
    toprollerTemperatureStatusSignal = m_topRollerMotor.getDeviceTemp();

    bottomrollerVoltageStatusSignal = m_bottomRollerMotor.getMotorVoltage();
    toprollerVoltageStatusSignal = m_topRollerMotor.getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0,
        bottomrollerVelocityStatusSignal,
        toprollerVelocityStatusSignal);
        BaseStatusSignal.setUpdateFrequencyForAll(10.0,
        bottomrollerTorqueCurrentStatusSignal,
        toprollerTorqueCurrentStatusSignal,
        bottomrollerSupplyCurrentStatusSignal,
        toprollerSupplyCurrentStatusSignal,
        bottomrollerReferenceVelocityStatusSignal,
        toprollerReferenceVelocityStatusSignal,
        bottomrollerTemperatureStatusSignal,
        toprollerTemperatureStatusSignal,
        bottomrollerVoltageStatusSignal,
        toprollerVoltageStatusSignal);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {

    BaseStatusSignal.refreshAll(
      bottomrollerVelocityStatusSignal,
      bottomrollerTorqueCurrentStatusSignal,
      bottomrollerSupplyCurrentStatusSignal,
      bottomrollerReferenceVelocityStatusSignal,
      toprollerSupplyCurrentStatusSignal,
      toprollerTorqueCurrentStatusSignal,
      toprollerVelocityStatusSignal,
      toprollerReferenceVelocityStatusSignal,
      bottomrollerTemperatureStatusSignal,
      toprollerTemperatureStatusSignal,
      bottomrollerVoltageStatusSignal,
      toprollerVoltageStatusSignal);

    inputs.bottomrollerTorqueCurrentAmps = bottomrollerTorqueCurrentStatusSignal.getValueAsDouble();
    inputs.toprollerTorqueCurrentAmps = toprollerTorqueCurrentStatusSignal.getValueAsDouble();

    inputs.bottomrollerSupplyCurrentAmps = bottomrollerSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.toprollerSupplyCurrentAmps = toprollerSupplyCurrentStatusSignal.getValueAsDouble();

    inputs.bottomrollerVelocityRPS = bottomrollerVelocityStatusSignal.getValueAsDouble();
    inputs.toprollerVelocityRPS = toprollerVelocityStatusSignal.getValueAsDouble();

    // Retrieve the closed loop reference status signals directly from the motor in
    // this method
    // instead of retrieving in advance because the status signal returned depends
    // on the current
    // control mode.
    inputs.bottomrollerReferenceVelocityRPS = m_bottomRollerMotor.getClosedLoopReference().getValueAsDouble();
    inputs.toprollerReferenceVelocityRPS = m_topRollerMotor.getClosedLoopReference().getValueAsDouble();

    inputs.bottomrollerTempCelsius = bottomrollerTemperatureStatusSignal.getValueAsDouble();
    inputs.toprollerTempCelsius = toprollerTemperatureStatusSignal.getValueAsDouble();

    inputs.bottomrollerVoltage = bottomrollerVoltageStatusSignal.getValueAsDouble();
    inputs.toprollerVoltage = toprollerVoltageStatusSignal.getValueAsDouble();
  }

    @Override
    public void setBottomRollerVoltage(double volts) {
      m_bottomRollerMotor.setControl(bottomRollerVoltageSet.withOutput(volts));
    }

    @Override
    public void setTopRollerVoltage(double volts) {
      m_topRollerMotor.setControl(topRollerVoltageSet.withOutput(volts));
    }

    private void configurebottomRollerMotor(TalonFX rollerMotor) {
      // invert roller
      TalonFXConfiguration bottomrollerConfig = new TalonFXConfiguration();

      bottomrollerConfig.CurrentLimits.SupplyCurrentLimit = RollerConstants.SUPPLY_CURRENT_LIMIT_BOTTOM_ROLLER;
      bottomrollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

      bottomrollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

      bottomrollerConfig.Slot0.kP = 0;
      bottomrollerConfig.Slot0.kI = 0;
      bottomrollerConfig.Slot0.kD = 0;
      bottomrollerConfig.Slot0.kS = 0;

      // TODO: CORRECT LATER
      bottomrollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      // TODO: CHECK VALUE
      bottomrollerConfig.Feedback.SensorToMechanismRatio = 1;

      StatusCode status = StatusCode.StatusCodeNotInitialized;
      for (int i = 0; i < 5; ++i) {
        status = rollerMotor.getConfigurator().apply(bottomrollerConfig);
        if (status.isOK())
          break;
      }
    }

    private void configureTopRoller(TalonFX topRoller) {
      TalonFXConfiguration topRollerConfig = new TalonFXConfiguration();

      topRollerConfig.CurrentLimits.SupplyCurrentLimit = RollerConstants.SUPPLY_CURRENT_LIMIT_TOP_ROLLER;
      topRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

      topRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      // TODO: CORRECT LATER
      topRollerConfig.Slot0.kP = 0;
      topRollerConfig.Slot0.kI = 0;
      topRollerConfig.Slot0.kD = 0;
      topRollerConfig.Slot0.kS = 0;

      // TODO: CHANGE LATER
      topRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      topRollerConfig.Feedback.SensorToMechanismRatio = 1;

      StatusCode status = StatusCode.StatusCodeNotInitialized;
      for (int i = 0; i < 5; ++i) {
        status = topRoller.getConfigurator().apply(topRollerConfig);
        if (status.isOK())
          break;
      }
    }
  }
