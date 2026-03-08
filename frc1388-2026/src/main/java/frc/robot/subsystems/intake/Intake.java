// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
// import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
  /** Creates a new RollerSubsystem. */

  private final IntakeIO m_io;

  private final IntakeMotor[] m_intakeMotors = {
    IntakeMotor.DEPLOYMOTOR, IntakeMotor.ROLLERMOTOR,
  };

  private BooleanSupplier m_isReadyToIntake;
  
  public IntakeState intakeState;

  private final IntakeIOInputs inputs = new IntakeIOInputs();

  private BooleanSupplier coastOverride = () -> false;

  enum IntakeMotor {
    DEPLOYMOTOR,
    ROLLERMOTOR
  }

  public enum IntakeState {
    RETRACT,
    EXTENDED,
    INTAKING,
    SHOOTING,
    MEDIUS,
    TESTINGROLLER,
    TESTINGDEPLOY,
    STOP
  }

  
  private final MutVoltage sysidAppliedVoltageMeasure = Volts.mutable(0);
  private final MutAngle sysidPositionMeasure = Rotations.mutable(0);
  private final MutAngularVelocity sysidVelocityMeasure = RotationsPerSecond.mutable(0);

  private final SysIdRoutine sysIdRoutine;  

  public Intake(IntakeIO io) {
    m_io = io;
    intakeState = IntakeState.RETRACT;

    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(3), null, null),
        new SysIdRoutine.Mechanism(
            (Voltage volts) -> {
              setDeployVolts(volts.in(Volts));
            },
            log -> {
              log.motor("intake-deploy")
                  .voltage(sysidAppliedVoltageMeasure.mut_replace(inputs.deployMotorVoltage,
                      Volts))
                  .angularPosition(
                      sysidPositionMeasure.mut_replace(inputs.deployMotorPosition,
                          Rotations))
                  .angularVelocity(sysidVelocityMeasure.mut_replace(inputs.deployMotorReferenceVelocityRPS,
                      RotationsPerSecond));
            },
            this));
  }

  public void periodic() {
    m_io.updateInputs(inputs);
    if (intakeState == IntakeState.RETRACT) {
      setIntakingRollers(0);
      setPosition(IntakeConstants.UP_POSITION);
    } else if (intakeState == IntakeState.EXTENDED) {
      setPosition(IntakeConstants.DOWN_POSITION);
      setIntakingRollers(0);
    } else if (intakeState == IntakeState.INTAKING) {
      setPosition(IntakeConstants.DOWN_POSITION);
      setIntakingRollers(IntakeConstants.INTAKING_ROLLER_STATE_VOLTS);
    } else if (intakeState == IntakeState.SHOOTING) {
      setPosition(IntakeConstants.HALF_WAY);
      setIntakingRollers(IntakeConstants.INTAKING_ROLLER_STATE_VOLTS);
    } else if (intakeState == IntakeState.TESTINGROLLER) {
      setIntakingRollers(IntakeConstants.TESTING_VOLTS);
    } else if (intakeState == IntakeState.TESTINGDEPLOY) {
      setDeployVolts(4);
    } else if (intakeState == IntakeState.STOP) {
      setIntakingRollers(0);
    }
  }

    public void stop() {
      m_io.setDeployVoltage(0);
      m_io.setRollerVoltage(0);
    }

    public void setIntakingRollers(double voltage) {
      m_io.setRollerVoltage(voltage);
    }

    public void setPosition(double position) {
      m_io.setDeployPosition(position);
    }

    public void setDeployVolts(double voltage) {
      m_io.setDeployVoltage(voltage);
    }

    public void setIntakeState(IntakeState intakeState) {
      this.intakeState = intakeState;
    }

    public IntakeState getIntakeState() {
      return intakeState;
    }

    public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("intake.sysIdQuasistatic");
    }

    public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("intake.sysIdDynamic");
    }
}