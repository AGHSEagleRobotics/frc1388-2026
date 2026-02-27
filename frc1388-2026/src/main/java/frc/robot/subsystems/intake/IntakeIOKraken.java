// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/** Add your docs here. */
public class IntakeIOKraken implements IntakeIO {
private TalonFX m_deployMotor;
  private TalonFX m_rollerMotor;

  private VoltageOut deployMotorVoltageSet;
  private VoltageOut rollerMotorVoltageSet;

  private StatusSignal<AngularVelocity> deployMotorVelocityStatusSignal;
  private StatusSignal<AngularVelocity> rollerMotorVelocityStatusSignal;

  private StatusSignal<Current> deployMotorTorqueCurrentStatusSignal;
  private StatusSignal<Current> rollerMotorTorqueCurrentStatusSignal;

  private StatusSignal<Current> deployMotorSupplyCurrentStatusSignal;
  private StatusSignal<Current> rollerMotorSupplyCurrentStatusSignal;

  private StatusSignal<Double> deployMotorReferenceVelocityStatusSignal;
  private StatusSignal<Double> rollerMotorReferenceVelocityStatusSignal;

  private StatusSignal<Temperature> deployMotorTemperatureStatusSignal;
  private StatusSignal<Temperature> rollerMotorTemperatureStatusSignal;

  private StatusSignal<Voltage> deployMotorVoltageStatusSignal;
  private StatusSignal<Voltage> rollerMotorVoltageStatusSignal;

  public IntakeIOKraken() {
    m_deployMotor = new TalonFX(0);
    m_rollerMotor = new TalonFX(0);

    configureDeployMotor(m_deployMotor);
    configurerollerMotor(m_rollerMotor);

    deployMotorVoltageSet = new VoltageOut(0);
    rollerMotorVoltageSet = new VoltageOut(0);

    deployMotorVelocityStatusSignal = m_deployMotor.getVelocity();
    rollerMotorVelocityStatusSignal = m_rollerMotor.getVelocity();

    deployMotorTorqueCurrentStatusSignal = m_deployMotor.getTorqueCurrent();
    rollerMotorTorqueCurrentStatusSignal = m_rollerMotor.getTorqueCurrent();

    deployMotorSupplyCurrentStatusSignal = m_deployMotor.getSupplyCurrent();
    rollerMotorSupplyCurrentStatusSignal = m_rollerMotor.getSupplyCurrent();

    deployMotorReferenceVelocityStatusSignal = m_deployMotor.getClosedLoopReference();
    rollerMotorReferenceVelocityStatusSignal = m_rollerMotor.getClosedLoopReference();

    deployMotorTemperatureStatusSignal = m_deployMotor.getDeviceTemp();
    rollerMotorTemperatureStatusSignal = m_rollerMotor.getDeviceTemp();

    deployMotorVoltageStatusSignal = m_deployMotor.getMotorVoltage();
    rollerMotorVoltageStatusSignal = m_rollerMotor.getMotorVoltage();
  }

   @Override
  public void updateInputs(IntakeIOInputs inputs) {

    BaseStatusSignal.refreshAll(
        deployMotorVelocityStatusSignal,
        deployMotorTorqueCurrentStatusSignal,
        deployMotorSupplyCurrentStatusSignal,
        deployMotorReferenceVelocityStatusSignal,
        rollerMotorSupplyCurrentStatusSignal,
        rollerMotorTorqueCurrentStatusSignal,
        rollerMotorVelocityStatusSignal,
        rollerMotorReferenceVelocityStatusSignal,
        deployMotorTemperatureStatusSignal,
        rollerMotorTemperatureStatusSignal,
        deployMotorVoltageStatusSignal,
        rollerMotorVoltageStatusSignal);


    inputs.deployMotorTorqueCurrentAmps = deployMotorTorqueCurrentStatusSignal.getValueAsDouble();
    inputs.rollerMotorTorqueCurrentAmps = rollerMotorTorqueCurrentStatusSignal.getValueAsDouble();

    inputs.deployMotorSupplyCurrentAmps = deployMotorSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.rollerMotorSupplyCurrentAmps = rollerMotorSupplyCurrentStatusSignal.getValueAsDouble();

    inputs.bottomrollerVelocityRPS = bottomrollerVelocityStatusSignal.getValueAsDouble();
    inputs.toprollerVelocityRPS = toprollerVelocityStatusSignal.getValueAsDouble();

    // Retrieve the closed loop reference status signals directly from the motor in this method
    // instead of retrieving in advance because the status signal returned depends on the current
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
    TalonFXConfiguration bottomrollerConfig = new TalonFXConfiguration();
    TorqueCurrentConfigs bottomrollerTorqueCurrentConfigs = new TorqueCurrentConfigs();

    // TODO: CHANGE LATER
    bottomrollerTorqueCurrentConfigs.PeakForwardTorqueCurrent = 0;
    bottomrollerTorqueCurrentConfigs.PeakReverseTorqueCurrent = 0;

    bottomrollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    bottomrollerConfig.Slot0.kP = 0;
    bottomrollerConfig.Slot0.kI = 0;
    bottomrollerConfig.Slot0.kD = 0;
    bottomrollerConfig.Slot0.kS = 0;

    //TODO: CORRECT LATER
    bottomrollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    //TODO: CHECK VALUE
    bottomrollerConfig.Feedback.SensorToMechanismRatio = 1;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = rollerMotor.getConfigurator().apply(bottomrollerConfig);
      if (status.isOK()) break;
    }
  }

  private void configureTopRoller(TalonFX topRoller) {
    TalonFXConfiguration topRollerConfig = new TalonFXConfiguration();
    TorqueCurrentConfigs toprollerTorqueCurrentConfigs = new TorqueCurrentConfigs();

    //TODO: CORRECT LATER
    toprollerTorqueCurrentConfigs.PeakForwardTorqueCurrent = 0;
    toprollerTorqueCurrentConfigs.PeakReverseTorqueCurrent = 0;

    topRollerConfig.TorqueCurrent = toprollerTorqueCurrentConfigs;

    topRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // TODO: CORRECT LATER
    topRollerConfig.Slot0.kP = 0;
    topRollerConfig.Slot0.kI = 0;
    topRollerConfig.Slot0.kD = 0;
    topRollerConfig.Slot0.kS = 0;

    //TODO: CHANGE LATER
    topRollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    topRollerConfig.Feedback.SensorToMechanismRatio = 1;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = topRoller.getConfigurator().apply(topRollerConfig);
      if (status.isOK()) break;
    }
  }
}
