package frc.robot.subsystems;


import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.DriverStation;

public class LEDSubsystem extends SubsystemBase {
  private DriverStation.Alliance lastAlliance = null;

  private final CANdle m_candle; // CANdle canid is 42
  // private final boolean m_isOnRed;

  // LED Constants

  private static final int kSlotStart = 8;
  private static final int kSlotEnd = 50;
  CANdleConfiguration configOn;
  CANdleConfiguration configOff;

  /** Creates a new LEDSubsystem. */  
  public LEDSubsystem(CANdle candle) {

    m_candle = candle;

    // m_isOnRed = (DriverStation.getAlliance().get() == Alliance.Red);

    configOn = new CANdleConfiguration();
    configOn.withLED(new LEDConfigs().withStripType(StripTypeValue.GRB).withBrightnessScalar(0.7));
    

    configOff = new CANdleConfiguration();
    configOff.withLED(new LEDConfigs().withStripType(StripTypeValue.GRB).withBrightnessScalar(0));
    

    // setSolidWhite();

  } // end LEDSubsystem() constructor



  public void setSolidWhite(){
    m_candle.getConfigurator().apply(configOn);
    m_candle.setControl(
        new SolidColor(13, 45)
          .withColor(new RGBWColor(Color.kWhite).scaleBrightness(0.1))
          );
  }

  public void turnOffSolidWhite(){
    m_candle.getConfigurator().apply(configOff);
  } 

  public void setSolidNoTeam(){
    m_candle.getConfigurator().apply(configOn);
    m_candle.setControl(
      new SolidColor(8, 12)
        .withColor(new RGBWColor(Color.kGreen).scaleBrightness(0.5))
    );

    m_candle.setControl(
      new SolidColor(46, 50)
        .withColor(new RGBWColor(Color.kGreen).scaleBrightness(0.5))
    );
  }

  public void setSolidBlueTeam(){
    m_candle.getConfigurator().apply(configOn);
    m_candle.setControl(
        new SolidColor(8, 12)
          .withColor(new RGBWColor(Color.kDarkBlue).scaleBrightness(0.5))
    );

    m_candle.setControl(
      new SolidColor(46, 50)
          .withColor(new RGBWColor(Color.kDarkBlue).scaleBrightness(0.5))
    );
    

  }
  public void setSolidRedTeam(){
    m_candle.getConfigurator().apply(configOn);
     m_candle.setControl(
        new SolidColor(8, 12)
          .withColor(new RGBWColor(Color.kDarkRed).scaleBrightness(0.5))
    );

    m_candle.setControl(
        new SolidColor(46, 50)
          .withColor(new RGBWColor(Color.kDarkRed).scaleBrightness(0.5))
    );
  }

  public void setEagleColors(){
    m_candle.getConfigurator().apply(configOn);
    m_candle.setControl(
      new SolidColor(0, 0)
      .withColor(new RGBWColor(Color.kDarkCyan).scaleBrightness(1))
    );
    m_candle.setControl(
      new SolidColor(1, 1)
      .withColor(new RGBWColor(Color.kYellow).scaleBrightness(1))
    );
    m_candle.setControl(
      new SolidColor(2,2)
      .withColor(new RGBWColor (Color.kDarkCyan).scaleBrightness(1))
      
    );
    
    m_candle.setControl(
      new SolidColor(3, 3)
      .withColor(new RGBWColor(Color.kYellow).scaleBrightness(1))
    );
     m_candle.setControl(
      new SolidColor(4,4)
      .withColor(new RGBWColor (Color.kDarkCyan).scaleBrightness(1))
    );

     m_candle.setControl(
      new SolidColor(5, 5)
      .withColor(new RGBWColor(Color.kYellow).scaleBrightness(1))
    );

      m_candle.setControl(
      new SolidColor(6,6)
      .withColor(new RGBWColor (Color.kDarkCyan).scaleBrightness(1))      
    );

     m_candle.setControl(
      new SolidColor(7, 7)
      .withColor(new RGBWColor(Color.kYellow).scaleBrightness(1))
    );

  }

   public void updateAllianceLEDs(){
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() != lastAlliance) 
            {
                lastAlliance = alliance.get();
              switch (lastAlliance) {
                    case Red: 
                        setSolidRedTeam();
                        break;
                    case Blue: 
                        setSolidBlueTeam();
                        break;
                    default:
                        setSolidNoTeam();
                        break;
                }
            }
                      
        }

  // public void setFireAnimation(){
  //   // Fire animation
  //   m_candle.setControl(
  //       new FireAnimation(kSlotStart, kSlotEnd).withSlot(1)
  //           .withDirection(AnimationDirectionValue.Backward)
  //           .withCooling(0.4)
  //           .withSparking(0.5));
  // }

  // public void setRainbowAnimation(){
  //   m_candle.setControl(
  //       new RainbowAnimation(kSlotStart, kSlotEnd).withSlot(2)
  //           .withDirection(AnimationDirectionValue.Forward)
  //           .withFrameRate(200));
  // }

  // public void setStrobeAnimation(){

  // }

  // public void setFadeAnimation(){
  //   // Yellow-Green fade in animation
  //   SingleFadeAnimation fades = new SingleFadeAnimation(8, 46).withSlot(0)
  //       .withColor(new RGBWColor(247, 233, 0, 0).scaleBrightness(.5)).withFrameRate(0.2);
  //   m_candle.setControl(fades);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateAllianceLEDs();
    
  }
}