// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.controller.PIDController;
import java.util.HashMap;




public class Ale_ThrowerSubsystem extends SubsystemBase {
  private TalonFX m_throwerMotor1 = new TalonFX(1);
  private TalonFX m_throwerMotor2 = new TalonFX(2);
  private TalonFX m_hoodMotor = new TalonFX(3);
  private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(1);
  // Slot0Configs slot0Configs = new Slot0Configs();
  private static final double kP = 0.01;
  private static final double kI = 0.00;
  private static final double kD = 0.00;
  private static final PIDController m_pidController = new PIDController(kP, kI, kD);

  private static final double kSetpoint1 = 45.0;
  private static final double kSetpoint2 = 55.0;
  private static final double kSetpoint3 = 65.0;


    // final double kP = 0.01; // Example values; actual values require tuning
   
  /** Creates a new ExampleSubsystem. */
  public Ale_ThrowerSubsystem(TalonFX throwerMotor1, TalonFX throwerMotor2, TalonFX hoodMotor) {
  

    MotorOutputConfigs currentConfigs = new MotorOutputConfigs();
    currentConfigs
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withNeutralMode(NeutralModeValue.Brake);

      m_throwerMotor1.getConfigurator().apply(currentConfigs);

      m_throwerMotor2.getConfigurator().apply(currentConfigs);

      m_hoodMotor.getConfigurator().apply(currentConfigs);
    /*Use above in case motors are backwards */

    m_throwerMotor1 = throwerMotor1;
    m_throwerMotor2 = throwerMotor2;
    m_hoodMotor = hoodMotor;

    // TalonFXConfiguration configuration = new ();
  }
  public void setPower(double power) {
    m_throwerMotor1.setControl(m_dutyCycleRequest.withOutput(power));
    m_throwerMotor2.setControl(m_dutyCycleRequest.withOutput(power));
    m_hoodMotor.setControl(m_dutyCycleRequest.withOutput(power));
  
  }


  
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    m_throwerMotor1.setControl(m_dutyCycleRequest);
    m_throwerMotor2.setControl(m_dutyCycleRequest);
    // double currentPosition = m_hoodMotor.getPosition().getValueAsDouble();
    // double output = m_pidController.calculate(currentPosition, kSetpoint2);
    // m_hoodMotor.set(output);
    // Defaults to middle setpoint maybe
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  

  
}
