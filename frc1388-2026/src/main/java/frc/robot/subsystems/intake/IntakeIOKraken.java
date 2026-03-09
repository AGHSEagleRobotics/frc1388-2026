// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants;

/** Add your docs here. */
public class IntakeIOKraken implements IntakeIO {
  private TalonFX m_deployMotor1;
  private TalonFX m_deployMotor2;
  private TalonFX m_rollerMotor;
  private CANcoder CANcoder;

  private VoltageOut rollerMotorVoltageRequest;
  private VoltageOut deployMotorVoltageRequest;
  private final MotionMagicVoltage deployMotorPositionRequest;

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

  private StatusSignal<Angle> deployMotorPositionStatusSignal;


  private final NeutralOut neutralOut = new NeutralOut();

  private final Follower followRequest = new Follower(44, MotorAlignmentValue.Opposed);

  public IntakeIOKraken() {
    m_deployMotor1 = new TalonFX(44);
    m_deployMotor2 = new TalonFX(45);
    m_rollerMotor = new TalonFX(43);
    CANcoder = new CANcoder(52);

    configureDeployMotors(m_deployMotor1, m_deployMotor2);
    configureRollerMotor(m_rollerMotor);
    configureCANcoder(CANcoder);

    deployMotorVoltageRequest = new VoltageOut(0);
    rollerMotorVoltageRequest = new VoltageOut(0);
    deployMotorPositionRequest = new MotionMagicVoltage(0);


    deployMotorVelocityStatusSignal = m_deployMotor1.getVelocity();
    rollerMotorVelocityStatusSignal = m_rollerMotor.getVelocity();

    deployMotorTorqueCurrentStatusSignal = m_deployMotor1.getTorqueCurrent();
    rollerMotorTorqueCurrentStatusSignal = m_rollerMotor.getTorqueCurrent();

    deployMotorSupplyCurrentStatusSignal = m_deployMotor1.getSupplyCurrent();
    rollerMotorSupplyCurrentStatusSignal = m_rollerMotor.getSupplyCurrent();

    deployMotorReferenceVelocityStatusSignal = m_deployMotor1.getClosedLoopReference();
    rollerMotorReferenceVelocityStatusSignal = m_rollerMotor.getClosedLoopReference();

    deployMotorTemperatureStatusSignal = m_deployMotor1.getDeviceTemp();
    rollerMotorTemperatureStatusSignal = m_rollerMotor.getDeviceTemp();

    deployMotorVoltageStatusSignal = m_deployMotor1.getMotorVoltage();
    rollerMotorVoltageStatusSignal = m_rollerMotor.getMotorVoltage();

    deployMotorPositionStatusSignal = m_deployMotor1.getPosition();

    
    m_deployMotor2.setControl(followRequest);
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
        rollerMotorVoltageStatusSignal,
        deployMotorPositionStatusSignal);


    inputs.deployMotorTorqueCurrentAmps = deployMotorTorqueCurrentStatusSignal.getValueAsDouble();
    inputs.rollerMotorTorqueCurrentAmps = rollerMotorTorqueCurrentStatusSignal.getValueAsDouble();

    inputs.deployMotorSupplyCurrentAmps = deployMotorSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.rollerMotorSupplyCurrentAmps = rollerMotorSupplyCurrentStatusSignal.getValueAsDouble();

    inputs.deployMotorVelocityRPS = deployMotorVelocityStatusSignal.getValueAsDouble();
    inputs.rollerMotorVelocityRPS = rollerMotorVelocityStatusSignal.getValueAsDouble();

    // Retrieve the closed loop reference status signals directly from the motor in this method
    // instead of retrieving in advance because the status signal returned depends on the current
    // control mode.
    inputs.deployMotorReferenceVelocityRPS = m_deployMotor1.getClosedLoopReference().getValueAsDouble();
    inputs.rollerMotorReferenceVelocityRPS = m_rollerMotor.getClosedLoopReference().getValueAsDouble();

    inputs.bottomrollerTempCelsius = deployMotorTemperatureStatusSignal.getValueAsDouble();
    inputs.rollerMotorTempCelsius = rollerMotorTemperatureStatusSignal.getValueAsDouble();

    inputs.deployMotorVoltage = deployMotorVoltageStatusSignal.getValueAsDouble();
    inputs.rollerMotorVoltage = rollerMotorVoltageStatusSignal.getValueAsDouble();

    inputs.deployMotorPosition = deployMotorPositionStatusSignal.getValueAsDouble();
    }
    @Override
    public void setDeployPosition(double position) {
      m_deployMotor1.setControl(deployMotorPositionRequest.withPosition(position));
    }

    @Override
    public void setDeployVoltage(double volts) {
        m_deployMotor1.setControl(deployMotorVoltageRequest.withOutput(volts));
    }

    @Override
    public void setRollerVoltage(double volts) {
      m_rollerMotor.setControl(rollerMotorVoltageRequest.withOutput(volts));
    }

    @Override
    public void stopRack() {
        m_deployMotor1.setControl(neutralOut);
    }
    
    @Override
    public void stopSpin() {
      m_rollerMotor.setControl(neutralOut);
    }
    
    @Override
    public void zeroPosition() {
      m_deployMotor1.setPosition(0);
    }
    
    public double getPosition() {
      return CANcoder.getAbsolutePosition().getValueAsDouble();
    }
    
  private void configureDeployMotors(TalonFX deployMotor1, TalonFX deployMotor2) {
    TalonFXConfiguration deployMotorConfig = new TalonFXConfiguration();

    deployMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT_DEPLOY;
    deployMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    deployMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // TODO: CORRECT LATER
    deployMotorConfig.Slot0.kG = 0;
    deployMotorConfig.Slot0.kV = 0;
    deployMotorConfig.Slot0.kA = 0;
    deployMotorConfig.Slot0.kP = 0;
    deployMotorConfig.Slot0.kI = 0;
    deployMotorConfig.Slot0.kD = 0;
    deployMotorConfig.Slot0.kS = 0;

    //TODO: CHANGE LATER
    deployMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    deployMotorConfig.Feedback.FeedbackSensorSource = 
    FeedbackSensorSourceValue.RemoteCANcoder;
    deployMotorConfig.Feedback.FeedbackRemoteSensorID = 52;
    deployMotorConfig.Feedback.SensorToMechanismRatio = 1;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = deployMotor1.getConfigurator().apply(deployMotorConfig);
      status = deployMotor2.getConfigurator().apply(deployMotorConfig);
      if (status.isOK()) break;
    }
  }

  private void configureRollerMotor(TalonFX rollerMotor) {
    TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();

    rollerMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT_ROLLER;
    rollerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

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

  public void configureCANcoder(CANcoder CANcoder) {
    CANcoderConfiguration CANcoderConfig = new CANcoderConfiguration();

    //TODO: CHANGE LATER
    CANcoderConfig.MagnetSensor.MagnetOffset = IntakeConstants.INTAKE_OFFSET;
       
    // Gives 0.0 to 1.0 range (full rotation, no discontinuity)
    CANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    
    CANcoder.getConfigurator().apply(CANcoderConfig);
  }
}
