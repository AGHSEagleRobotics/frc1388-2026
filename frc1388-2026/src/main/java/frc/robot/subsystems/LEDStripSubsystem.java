// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;



public class LEDStripSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
 private final CANdle m_candle;
 private static final int stripStart = 1;
 private static final int stripfin = 2; 
 // 1 and 2 are just placeholders for now. Please make sure to update these fields with the correct integers.
 CANdleConfiguration configOn;
 CANdleConfiguration configOff;

 
  public LEDStripSubsystem(CANdle LEDcandle) {
    m_LEDcandle = LEDcandle; 

    configOn = new CANdleConfiguration(); 
    configOn.withLED(new LEDConfigs().withStripType(StripTypeValue.RGB).withBrightnessScalar(0.45));

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
