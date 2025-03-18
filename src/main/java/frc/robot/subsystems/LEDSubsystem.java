// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.PowerJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDValues;
import frc.robot.Robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  //CANdle LED = new CANdle(LEDConstants.ID);
  CANdle LED = new CANdle(LEDConstants.ID, RobotContainer.MainBus.getName());

  int numLED = LEDValues.numLED;

  int baLEDLeft = 1; //Boot Animation LED Left
  int baLEDRight = numLED; //Boot ANimated LED Right
  boolean baCalledRedFade = false;
  double baLEDR = LEDValues.teamR;
  double baLEDB = LEDValues.teamB;
  double baLEDG = LEDValues.teamG;

  double bootDelay = 60;


  SingleFadeAnimation redFadeAnim;
  TwinkleAnimation twinkleAnim;

  RainbowAnimation rainbow = new RainbowAnimation(255, 0.2, 999);


  public LEDSubsystem() {
    setLEDs(255, 0, 0);
    redFadeAnim = new SingleFadeAnimation(LEDValues.teamR, LEDValues.teamG, LEDValues.teamB);
    redFadeAnim.setSpeed(0.5);
    redFadeAnim.setNumLed(numLED);
    twinkleAnim = new TwinkleAnimation(12, 250, 140);
    twinkleAnim.setNumLed(numLED);
    twinkleAnim.setSpeed(0.05);
    
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
    double batteryVoltage = PowerJNI.getVinVoltage();
    if(batteryVoltage < 7) //That one thing that sees the drop in battery voltage and stops LEDs if low uwu
    {
      RobotContainer.forceLEDoff = true;
      setLEDs(0, 0, 0);
      //System.out.println("AHHH!!! BATTERY DIPPING!!!!");
    }

    if(Robot.runBootAnimation)
    {
      if(bootDelay < 0)
      {
        if(baLEDLeft < baLEDRight)
        {
          LED.setLEDs((int)baLEDR, (int)baLEDG, (int)baLEDB, 0, baLEDLeft, 1);
          LED.setLEDs((int)baLEDR, (int)baLEDG, (int)baLEDB, 0, baLEDRight, 1);
          baLEDLeft += 0.015;
          baLEDRight -= 0.015;
          //System.out.println("left: " + Double.toString(baLEDLeft) + ", right: " + Double.toString(baLEDRight));
        }
        else
        {
          if(!baCalledRedFade)
          {
            baLEDR = MathUtil.interpolate(baLEDR, 0, 0.06); //dimming time baby
            baLEDG = MathUtil.interpolate(baLEDG, 0, 0.06);
            baLEDB = MathUtil.interpolate(baLEDB, 0, 0.06);
            
            baLEDR = MathUtil.clamp(baLEDR, 0, 255); //clamping time baby
            baLEDB = MathUtil.clamp(baLEDB, 0, 255);
            baLEDG = MathUtil.clamp(baLEDG, 0, 255);
  
            setLEDs((int)baLEDR, (int)baLEDG, (int)baLEDB);
            
            if(baLEDR+baLEDG+baLEDB <= 1) //Haha look at me doing something partially smart
            {
              animRedFade();
              baCalledRedFade = true;
            }
          }
        }
      }
      else
      {
        bootDelay -= 1;
        setOff();
      }
      
    }
    if(RobotContainer.forceLEDoff)
    {
      setLEDs(0, 0, 0);
    }
    //setErrors();
  } 

  public void setLEDs(int R, int B, int G)
  {
    clearAnimation();
    LED.setLEDs(R, G, B, 255, 0  , numLED-10);
    //int ledRestLength = numLED - 210;
    //LED.setLEDs(R, G, B, 255, numLED-10, 10);

    



  }

  public void setErrors()
  {
    LED.setLEDs(0, 255, 255, 255, numLED-10, 10);

  }

  public void animRedFade()
  {
    if(!RobotContainer.forceLEDoff)
    {
      clearAnimation();
      LED.animate(redFadeAnim);
      
    }
  }
  public void setOff()
  {
    clearAnimation();
    LED.setLEDs(0, 0, 0);
  }
  public void setRed()
  {
    if(!RobotContainer.forceLEDoff)
    {
      clearAnimation();
      LED.setLEDs(255, 0, 0);
    }
    
  }
  public void setGreen()
  {
    if(!RobotContainer.forceLEDoff)
    {
      clearAnimation();
      LED.setLEDs(0, 255, 0);
    }
    
  }
  public void setPurple()
  {
    if(!RobotContainer.forceLEDoff)
    {
      clearAnimation();
      LED.setLEDs(255, 0, 255);
    }
  }
  public void setTeal()
  {
    if(!RobotContainer.forceLEDoff)
    {
      clearAnimation();
      LED.setLEDs(0, 255, 100);
    }
  }
  public void setDarkRed()
  {
    if(!RobotContainer.forceLEDoff)
    {
      clearAnimation();
      LED.setLEDs(100, 0, 0);
    }
  }
  public void setTwinkle()
  {
    if(!RobotContainer.forceLEDoff)
    {
      clearAnimation();
      LED.animate(twinkleAnim);
    } 
  }
  public void setRainbow()
  {
    if(!RobotContainer.forceLEDoff)
    {
      LED.animate(rainbow);
    }
  }
  public void clearAnimation()
  {
    LED.clearAnimation(0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
