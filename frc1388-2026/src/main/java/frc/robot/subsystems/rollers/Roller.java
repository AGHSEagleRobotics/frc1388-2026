// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.rollers.RollerIO.RollerIOInputs;
import frc.robot.subsystems.rollers.RollerIO.RollerIOMode;
import frc.robot.subsystems.rollers.RollerIO.RollerIOOutputs;

public class Roller extends SubsystemBase {
  /** Creates a new RollerSubsystem. */

  private final RollerIO m_io;

  private final RollerMotor[] m_rollerMotors = {
    RollerMotor.BOTTOMROLLER, RollerMotor.TOPROLLER,
  };

  private BooleanSupplier m_isReadyToShoot;
  
  public RollerState rollerState;

  private final RollerIOInputs inputs = new RollerIOInputs();
  private final RollerIOOutputs outputs = new RollerIOOutputs();

  private BooleanSupplier coastOverride = () -> false;

  enum RollerMotor {
    TOPROLLER,
    BOTTOMROLLER
  }

  enum RollerState {
    BRAKE,
    INTAKING,
    SHOOTING
  }

  public Roller(RollerIO io) {
    m_io = io;
  }

  public void setShootingReady(BooleanSupplier isReadyToShoot) {
    m_isReadyToShoot = isReadyToShoot;
  }

  public void periodic() {
    m_io.updateInputs(inputs);
    if (rollerState == RollerState.BRAKE) {
      stop();
    }
    else if(rollerState == RollerState.INTAKING) {
      setIntakingRollers();
    }
    else if ((rollerState == RollerState.SHOOTING) && (m_isReadyToShoot.getAsBoolean())) {
      setShootingRollers();
    }

    }

    public void stop() {
      m_io.setBottomRollerVoltage(0);
      m_io.setTopRollerVoltage(0);
    }

    public void setIntakingRollers() {
      m_io.setBottomRollerVoltage(RollerConstants.bottomRollerIntakeSpeed);
    }

    public void setShootingRollers() {
      m_io.setBottomRollerVoltage(RollerConstants.bottomRollerShootingSpeed);
      m_io.setTopRollerVoltage(RollerConstants.topRollerShootingSpeed);
    }
}