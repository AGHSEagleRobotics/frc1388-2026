// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
// import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOMode;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;

public class Intake extends SubsystemBase {
  /** Creates a new RollerSubsystem. */

  private final IntakeIO m_io;

  private final IntakeMotor[] m_intakeMotors = {
    IntakeMotor.DEPLOYMOTOR, IntakeMotor.ROLLERMOTOR,
  };

  private BooleanSupplier m_isReadyToIntake;
  
  public IntakeState intakeState;

  private final IntakeIOInputs inputs = new IntakeIOInputs();
  private final IntakeIOOutputs outputs = new IntakeIOOutputs();

  private BooleanSupplier coastOverride = () -> false;

  enum IntakeMotor {
    DEPLOYMOTOR,
    ROLLERMOTOR
  }

  public enum IntakeState {
    RETRACTED,
    EXTENDED,
    INTAKING,
    AGITATING,
    MEDIUS,
  }

  public Intake(IntakeIO io) {
    m_io = io;
    intakeState = IntakeState.RETRACTED;
  }

  public void setIntakingReady(BooleanSupplier isReadyToIntake) {
    m_isReadyToIntake = isReadyToIntake;
  }

  public void periodic() {
    m_io.updateInputs(inputs);
    if (intakeState == IntakeState.RETRACTED) {
      stop();
    }
    else if(intakeState == IntakeState.EXTENDED) {
      setDeployMotor();
    }
    else if ((intakeState == IntakeState.INTAKING) && (m_isReadyToIntake.getAsBoolean())) {
      setIntakingRollers();
    }
    else if (intakeState == IntakeState.AGITATING) {
      setDeployMotor();

    }

    }

    public void stop() {
      m_io.setDeployMotorVoltage(0);
      m_io.setRollerMotorVoltage(0);
    }

    public void setIntakingRollers() {
      m_io.setDeployMotorVoltage(Constants.deployMotorSpeed);
    }

    public void setIntakingRollers() {
      m_io.setBottomRollerVoltage(Constants.bottomRollerShootingSpeed);
      m_io.setTopRollerVoltage(RConstants.topRollerShootingSpeed);
    }

    public RollerState getRollerState() {
      return rollerState;
    }

    public void setRollerState(RollerState rollerState) {
      this.rollerState = rollerState;
    }
}