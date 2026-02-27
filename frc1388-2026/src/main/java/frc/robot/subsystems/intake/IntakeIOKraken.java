// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
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

  public static final double ROTOR_TO_PINION_RATIO = 2.0 / 1;
  public static final Distance PINION_PITCH_RADIUS = Inches.of(0.5);

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

  private final MotionMagicVoltage rackPositionRequest = new MotionMagicVoltage(0);
  private final VoltageOut rackVoltageRequest = new VoltageOut(0);
  private final VoltageOut spinVoltageRequest = new VoltageOut(0).withEnableFOC(false);

  private final NeutralOut neutralOut = new NeutralOut();


  public IntakeIOKraken() {
    m_deployMotor = new TalonFX(0);
    m_rollerMotor = new TalonFX(0);

    configureDeployMotor(m_deployMotor);
    configureRollerMotor(m_rollerMotor);

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

    inputs.deployMotorVelocityRPS = deployMotorVelocityStatusSignal.getValueAsDouble();
    inputs.rollerMotorVelocityRPS = rollerMotorVelocityStatusSignal.getValueAsDouble();

    // Retrieve the closed loop reference status signals directly from the motor in this method
    // instead of retrieving in advance because the status signal returned depends on the current
    // control mode.
    inputs.deployMotorReferenceVelocityRPS = m_deployMotor.getClosedLoopReference().getValueAsDouble();
    inputs.rollerMotorReferenceVelocityRPS = m_rollerMotor.getClosedLoopReference().getValueAsDouble();

    inputs.bottomrollerTempCelsius = deployMotorTemperatureStatusSignal.getValueAsDouble();
    inputs.rollerMotorTempCelsius = rollerMotorTemperatureStatusSignal.getValueAsDouble();

    inputs.deployMotorVoltage = deployMotorVoltageStatusSignal.getValueAsDouble();
    inputs.rollerMotorVoltage = rollerMotorVoltageStatusSignal.getValueAsDouble();

    }

    public static Distance rotorAngleToDistance(Angle rotorAngle) {
        return PINION_PITCH_RADIUS.times(rotorAngle.in(Radians));
    }

    public static Angle distanceToRotorAngle(Distance distance) {
        return Radians.of(distance.in(Meters) / PINION_PITCH_RADIUS.in(Meters));
    }

    @Override
    public void setRackPosition(Distance position) {
      m_deployMotor.setControl(rackPositionRequest.withPosition(distanceToRotorAngle(position)));
    }

    @Override
    public void setRackOutput(Voltage out) {
        m_deployMotor.setControl(rackVoltageRequest.withOutput(out));
    }

    @Override
    public void setSpinOutput(Voltage volts) {
      m_rollerMotor.setControl(rollerMotorVoltageSet.withOutput(volts));
    }

    @Override
    public void stopRack() {
        m_deployMotor.setControl(neutralOut);
    }

    @Override
    public void stopSpin() {
        m_rollerMotor.setControl(neutralOut);
    }
    
    @Override
    public void zeroPosition() {
        m_deployMotor.setPosition(0);
    }

      private void configuredeployMotor(TalonFX deployMotor) {
    TalonFXConfiguration deployMotorConfig = new TalonFXConfiguration();
    TorqueCurrentConfigs deployMotorTorqueCurrentConfigs = new TorqueCurrentConfigs();

    // TODO: CHANGE LATER
    deployMotorTorqueCurrentConfigs.PeakForwardTorqueCurrent = 0;
    deployMotorTorqueCurrentConfigs.PeakReverseTorqueCurrent = 0;

    deployMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    deployMotorConfig.Slot0.kP = 0;
    deployMotorConfig.Slot0.kI = 0;
    deployMotorConfig.Slot0.kD = 0;
    deployMotorConfig.Slot0.kS = 0;

    //TODO: CORRECT LATER
    deployMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    //TODO: CHECK VALUE
    deployMotorConfig.Feedback.SensorToMechanismRatio = 1;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = deployMotor.getConfigurator().apply(deployMotorConfig);
      if (status.isOK()) break;
    }
  }

  private void configureDeployMotor(TalonFX deployMotor) {
    TalonFXConfiguration deployMotorConfig = new TalonFXConfiguration();
    TorqueCurrentConfigs deployMotorTorqueCurrentConfigs = new TorqueCurrentConfigs();

    //TODO: CORRECT LATER
    deployMotorTorqueCurrentConfigs.PeakForwardTorqueCurrent = 0;
    deployMotorTorqueCurrentConfigs.PeakReverseTorqueCurrent = 0;

    deployMotorConfig.TorqueCurrent = deployMotorTorqueCurrentConfigs;

    deployMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // TODO: CORRECT LATER
    deployMotorConfig.Slot0.kP = 0;
    deployMotorConfig.Slot0.kI = 0;
    deployMotorConfig.Slot0.kD = 0;
    deployMotorConfig.Slot0.kS = 0;

    //TODO: CHANGE LATER
    deployMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    deployMotorConfig.Feedback.SensorToMechanismRatio = 1;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = deployMotor.getConfigurator().apply(deployMotorConfig);
      if (status.isOK()) break;
    }
  }

  private void configureRollerMotor(TalonFX rollerMotor) {
    TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();
    TorqueCurrentConfigs rollerMotorTorqueCurrentConfigs = new TorqueCurrentConfigs();

    //TODO: CORRECT LATER
    rollerMotorTorqueCurrentConfigs.PeakForwardTorqueCurrent = 0;
    rollerMotorTorqueCurrentConfigs.PeakReverseTorqueCurrent = 0;

    rollerMotorConfig.TorqueCurrent = rollerMotorTorqueCurrentConfigs;

    rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // TODO: CORRECT LATER
    rollerMotorConfig.Slot0.kP = 0;
    rollerMotorConfig.Slot0.kI = 0;
    rollerMotorConfig.Slot0.kD = 0;
    rollerMotorConfig.Slot0.kS = 0;

    //TODO: CHANGE LATER
    rollerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rollerMotorConfig.Feedback.SensorToMechanismRatio = 1;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = rollerMotor.getConfigurator().apply(rollerMotorConfig);
      if (status.isOK()) break;
    }
  }
}
