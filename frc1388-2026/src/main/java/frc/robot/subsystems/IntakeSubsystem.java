// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final SparkFlex m_intakeMotor1;
  private final SparkFlex m_intakeMotor2;
  private final SparkMax m_rollerMotor;
  private final DigitalInput m_intakeOutLS;
  private final DigitalInput m_intakeInLS;

  public IntakeSubsystem(
    SparkMaxConfig configMax,
    SparkFlexConfig configFlex,
    DigitalInput intakeOutLS,
    DigitalInput intakeInLS,
    SparkFlex intakeMotor1,
    SparkFlex intakeMotor2,
    SparkMax rollerMotor) {

  // m_intakeOutLS = intakeOutLS;
  // m_intakeInLS = intakeInLS;
  // m_intakeMotor1 = intakeMotor1;
  // m_intakeMotor2 = intakeMotor2;
  // m_rollerMotor = rollerMotor;
  } public IntakeSubsystem(SparkFlex sparkFlex, SparkFlex sparkFlex2, SparkMax sparkMax, DigitalInput digitalInput,
      DigitalInput digitalInput2) {
    //TODO Auto-generated constructor stub
  }
  {
    m_intakeMotor1 = new SparkFlex(1, MotorType.kBrushless);
    // m_intakeMotor1.setInverted(false);
    m_intakeMotor2 = new SparkFlex(2, MotorType.kBrushless);
    // m_intakeMotor2.setInverted(false);
    m_rollerMotor = new SparkMax(3, MotorType.kBrushless);
    // m_rollerMotor.setInverted(false);
    m_intakeOutLS = new DigitalInput(1);
    m_intakeInLS = new DigitalInput(2);
    //Placeholder values on CANID or PWM
    
    // m_intakeMotor1.configure(configFlex, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // m_intakeMotor2.configure(configFlex, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // m_rollerMotor.configure(configMax, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void powerOn(double power){
    if (reachedIntakeOutLS() && power > 0) {
      power = 0;
    }

    if (reachedIntakeInLS() && power < 0){
      power = 0;
    }

    m_intakeMotor1.set(power);
    m_intakeMotor2.set(power);

  }

    private boolean reachedIntakeOutLS(){
      return m_intakeOutLS.get();
    }
    private boolean reachedIntakeInLS(){
      return m_intakeInLS.get();
    }

  public void run (double speed){
    m_intakeMotor1.set(speed);
    m_intakeMotor2.set(speed);
    m_rollerMotor.set(speed);
  }

  public void stop(){
    m_intakeMotor1.set(0.5);
    m_intakeMotor2.set(0.5);
    m_rollerMotor.set(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
