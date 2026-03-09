// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOKraken implements ShooterIO {

  //statussignals
  private StatusSignal<AngularVelocity> shootMotor1VelocitySS;
  private StatusSignal<Voltage> shootMotor1VoltageSS;
  private StatusSignal<Current> shootMotor1TorqueCurrentAmpsSS;
  private StatusSignal<Current> shootMotor1SupplyCurrentAmpsSS;
  private StatusSignal<Temperature> shootMotor1TempCelsiusSS;
  private StatusSignal<Angle> shootMotor1PositionStatusSignal;
  
  private StatusSignal<AngularVelocity> shootMotor2VelocitySS;
  private StatusSignal<Voltage> shootMotor2VoltageSS;
  private StatusSignal<Current> shootMotor2TorqueCurrentAmpsSS;
  private StatusSignal<Current> shootMotor2SupplyCurrentAmpsSS;
  private StatusSignal<Temperature> shootMotor2TempCelsiusSS;
  private StatusSignal<Angle> shootMotor2PositionStatusSignal;

  
  private StatusSignal<AngularVelocity> kickerMotorVelocitySS;
  private StatusSignal<Voltage> kickerMotorVoltageSS;
  private StatusSignal<Current> kickerMotorTorqueCurrentAmpsSS;
  private StatusSignal<Current> kickerMotorSupplyCurrentAmpsSS;
  private StatusSignal<Temperature> kickerMotorTempCelsiusSS;
  private StatusSignal<Angle> kickerPositionStatusSignal;

  //hardware/talons
  private TalonFX shootMotor1;
  private TalonFX shootMotor2;
  private TalonFX kickerMotor;

  //velocity
  private double shootMotor1Velocity;
  private double shootMotor2Velocity;
  private double kickerMotorVelocity;

  // Control
  private final Slot0Configs controllerConfig = new Slot0Configs();
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);
  private final Follower followRequest = new Follower(36, MotorAlignmentValue.Opposed);

  public ShooterIOKraken() {
    shootMotor1 = new TalonFX(ShooterConstants.SHOOT_MOTOR1_CANID);
    shootMotor2 = new TalonFX(ShooterConstants.SHOOT_MOTOR2_CANID);
    kickerMotor = new TalonFX(ShooterConstants.KICKER_MOTOR_CANID);

    //PIDS config
    controllerConfig.kP = 0.0;
    controllerConfig.kI = 0.0;
    controllerConfig.kD = 0.0;
    controllerConfig.kS = 0.0;
    controllerConfig.kV = 0.0;
    controllerConfig.kA = 0.0;
    
    // General config
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT_SHOOTER;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    //applying configs
    shootMotor1.getConfigurator().apply(shooterConfig, 1.0);
    shootMotor2.getConfigurator().apply(shooterConfig, 1.0);
    shootMotor1.getConfigurator().apply(controllerConfig, 1.0);
    shootMotor2.getConfigurator().apply(controllerConfig, 1.0);
    
    kickerMotor.getConfigurator().apply(shooterConfig, 1.0);
    kickerMotor.getConfigurator().apply(controllerConfig, 1.0);
   

    //SS at the end is just statussignal i was too lazy to write it out all the time
    shootMotor1VelocitySS = shootMotor1.getVelocity();
    shootMotor1VoltageSS = shootMotor1.getMotorVoltage();
    shootMotor1TorqueCurrentAmpsSS = shootMotor1.getTorqueCurrent();
    shootMotor1SupplyCurrentAmpsSS = shootMotor1.getSupplyCurrent();
    shootMotor1TempCelsiusSS = shootMotor1.getDeviceTemp();
    shootMotor1PositionStatusSignal = shootMotor1.getPosition();
    
    shootMotor2VelocitySS = shootMotor2.getVelocity();
    shootMotor2VoltageSS = shootMotor2.getMotorVoltage();
    shootMotor2TorqueCurrentAmpsSS = shootMotor2.getTorqueCurrent();
    shootMotor2SupplyCurrentAmpsSS = shootMotor2.getSupplyCurrent();
    shootMotor2TempCelsiusSS = shootMotor2.getDeviceTemp();
    shootMotor2PositionStatusSignal = shootMotor2.getPosition();

    kickerMotorVelocitySS = kickerMotor.getVelocity();
    kickerMotorVoltageSS = kickerMotor.getMotorVoltage();
    kickerMotorTorqueCurrentAmpsSS = kickerMotor.getTorqueCurrent();
    kickerMotorSupplyCurrentAmpsSS = kickerMotor.getSupplyCurrent();
    kickerMotorTempCelsiusSS = kickerMotor.getDeviceTemp();
    kickerPositionStatusSignal = kickerMotor.getPosition();

    // Keep velocity fast for closed-loop control
    BaseStatusSignal.setUpdateFrequencyForAll(50.0,
        shootMotor1VelocitySS,
        shootMotor2VelocitySS,
        kickerMotorVelocitySS);

    // Everything else - logging only, 10Hz is plenty
    BaseStatusSignal.setUpdateFrequencyForAll(10.0,
        shootMotor1VoltageSS,
        shootMotor1TorqueCurrentAmpsSS,
        shootMotor1SupplyCurrentAmpsSS,
        shootMotor1TempCelsiusSS,
        shootMotor1PositionStatusSignal,
        shootMotor2VoltageSS,
        shootMotor2TorqueCurrentAmpsSS,
        shootMotor2SupplyCurrentAmpsSS,
        shootMotor2TempCelsiusSS,
        shootMotor2PositionStatusSignal,
        kickerMotorVoltageSS,
        kickerMotorTorqueCurrentAmpsSS,
        kickerMotorSupplyCurrentAmpsSS,
        kickerMotorTempCelsiusSS,
        kickerPositionStatusSignal);

      shootMotor2.setControl(followRequest); //same thing as setshootervolts w/ motor inverison, also might wanna look into feedforward constants
  }
  @Override
  public void updateInputs(ShooterInputs inputs) {
    inputs.shootMotor1Connected = 
      BaseStatusSignal.refreshAll(
        shootMotor1VelocitySS,
        shootMotor1VoltageSS,
        shootMotor1TorqueCurrentAmpsSS,
        shootMotor1SupplyCurrentAmpsSS,
        shootMotor1TempCelsiusSS,
        shootMotor1PositionStatusSignal)
      .isOK();
    inputs.shootMotor2Connected = 
      BaseStatusSignal.refreshAll(
        shootMotor2VelocitySS,
        shootMotor2VoltageSS,
        shootMotor2TorqueCurrentAmpsSS,
        shootMotor2SupplyCurrentAmpsSS,
        shootMotor2TempCelsiusSS,
        shootMotor2PositionStatusSignal)
      .isOK();
    inputs.kickerMotorConnected =
      BaseStatusSignal.refreshAll(
        kickerMotorVelocitySS,
        kickerMotorVoltageSS,
        kickerMotorTorqueCurrentAmpsSS,
        kickerMotorSupplyCurrentAmpsSS,
        kickerMotorTempCelsiusSS,
        kickerPositionStatusSignal)
      .isOK();

    //setting signals / updating motor inputs
    inputs.shootMotor1VelocityRPS = shootMotor1VelocitySS.getValueAsDouble();
    inputs.shootMotor1ReferenceVelocityRPS = this.shootMotor1Velocity;
    inputs.shootMotor1ClosedLoopReferenceRPS = shootMotor1.getClosedLoopReference().getValueAsDouble();
    inputs.shootMotor1Voltage = shootMotor1VoltageSS.getValueAsDouble();
    inputs.shootMotor1TorqueCurrentAmps = shootMotor1TorqueCurrentAmpsSS.getValueAsDouble();
    inputs.shootMotor1SupplyCurrentAmps = shootMotor1SupplyCurrentAmpsSS.getValueAsDouble();
    inputs.shootMotor1TempCelsius = shootMotor1TempCelsiusSS.getValueAsDouble();
    inputs.shootMotor1Position = shootMotor1PositionStatusSignal.getValueAsDouble();

    inputs.shootMotor2VelocityRPS = shootMotor2VelocitySS.getValueAsDouble();
    inputs.shootMotor2ReferenceVelocityRPS = this.shootMotor2Velocity;
    inputs.shootMotor2ClosedLoopReferenceRPS = shootMotor2.getClosedLoopReference().getValueAsDouble();
    inputs.shootMotor2Voltage = shootMotor2VoltageSS.getValueAsDouble();
    inputs.shootMotor2TorqueCurrentAmps = shootMotor2TorqueCurrentAmpsSS.getValueAsDouble();
    inputs.shootMotor2SupplyCurrentAmps = shootMotor2SupplyCurrentAmpsSS.getValueAsDouble();
    inputs.shootMotor2TempCelsius = shootMotor2TempCelsiusSS.getValueAsDouble();
    inputs.shootMotor2Position = shootMotor2PositionStatusSignal.getValueAsDouble();
    
    inputs.kickerMotorVelocityRPS = kickerMotorVelocitySS.getValueAsDouble();
    inputs.kickerMotorReferenceVelocityRPS = this.kickerMotorVelocity;
    inputs.kickerMotorClosedLoopReferenceRPS = kickerMotor.getClosedLoopReference().getValueAsDouble();
    inputs.kickerMotorVoltage = kickerMotorVoltageSS.getValueAsDouble();
    inputs.kickerMotorTorqueCurrentAmps = kickerMotorTorqueCurrentAmpsSS.getValueAsDouble();
    inputs.kickerMotorSupplyCurrentAmps = kickerMotorSupplyCurrentAmpsSS.getValueAsDouble();
    inputs.kickerMotorTempCelsius = kickerMotorTempCelsiusSS.getValueAsDouble();
    inputs.kickerPosition = kickerPositionStatusSignal.getValueAsDouble();
  }
  @Override
      public void setShooterVelocity(double shooterRPS) {
        shootMotor1.setControl(velocityControl.withVelocity(shooterRPS));
      }
  @Override
    public void setShooterVolts(double shootMotorVolts) {
      shootMotor1.setControl(voltageControl.withOutput(shootMotorVolts));
    }
  @Override
    public void setKickerVolts(double kickerVolts) {
      kickerMotor.setControl(voltageControl.withOutput(kickerVolts));
    }

  @Override
  public void setKickerVelocity(double kickerRPS) {
    kickerMotor.setControl(velocityControl.withVelocity(kickerRPS));
  }

  @Override
    public void stopShooter() {
      shootMotor1.setControl(neutralControl);
      kickerMotor.setControl(neutralControl);
    }
  @Override
    public void setCoastMode(boolean coast) {

    }
  }
   
  // @Override
  //   public void setVoltsMotor1(double volts) {
  //     shootMotor1.setControl(shootMotor1VoltageRequest.withOutput(volts));
  //   }
  //   public void setVoltsMotor2(double volts) {
  //     shootMotor1.setControl(shootMotor2VoltageRequest.withOutput(volts));
  //   }

  