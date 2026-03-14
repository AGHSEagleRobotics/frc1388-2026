// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

public interface ShooterIO {
 public static class ShooterInputs {
  //VelocityRPS is the actual speed of the motor in revolutions per second,
  //ReferenceVelocityRPS is the velocity that we are asking for it to be, 
  //and ClosedLoopReferenceRPS is the velocity that the motor is trying to reach at that moment
  public boolean shootMotor1Connected = true;
  public boolean shootMotor2Connected = true;
  public boolean kickerMotorConnected = true;

  public double shootMotor1VelocityRPS = 0.0;
  public double shootMotor2VelocityRPS = 0.0;
  public double kickerMotorVelocityRPS = 0.0;

  public double shootMotor1ReferenceVelocityRPS = 0.0;
  public double shootMotor2ReferenceVelocityRPS = 0.0;
  public double kickerMotorReferenceVelocityRPS = 0.0;

  public double shootMotor1ClosedLoopReferenceRPS = 0.0;
  public double shootMotor2ClosedLoopReferenceRPS = 0.0;
  public double kickerMotorClosedLoopReferenceRPS = 0.0;

  public double shootMotor1Voltage = 0.0;
  public double shootMotor2Voltage = 0.0;
  public double kickerMotorVoltage = 0.0;

  public double shootMotor1TorqueCurrentAmps = 0.0;
  public double shootMotor2TorqueCurrentAmps = 0.0;
  public double kickerMotorTorqueCurrentAmps = 0.0;
  
  public double shootMotor1SupplyCurrentAmps = 0.0;
  public double shootMotor2SupplyCurrentAmps = 0.0;
  public double kickerMotorSupplyCurrentAmps = 0.0;

  public double shootMotor1TempCelsius = 0.0;
  public double shootMotor2TempCelsius = 0.0;
  public double kickerMotorTempCelsius = 0.0;

  public double shootMotor1Position = 0.0;
  public double shootMotor2Position = 0.0;
  public double kickerPosition = 0.0;
 }

  public void updateInputs(ShooterInputs inputs);

  public void setShooterVelocity(double shooterRPS);
    
  public void setShooterVolts(double shootMotorVolts);

  public void setKickerVolts(double kickerVolts);
  
  public void setKickerVelocity(double kickerRPS);

  public void stopShooter();

  public void setCoastMode(boolean coast);
  
  }

