// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.PowerJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  
  //Sets up the CANdle object (CANdle is the LED Controller)
  CANdle LED = new CANdle(LEDConstants.ID, RobotContainer.MainBus.getName()); 
  //We use .getName() on MainBus since CANdle() doesnt except a CANBUS object, but a string of its name.


  

  int numLED = LEDValues.numLED; 

  //These are for the fancy LED animation that doesnt work properly
  int baLEDLeft = 1; //Boot Animation LED Left
  int baLEDRight = numLED; //Boot ANimated LED Right
  boolean baCalledRedFade = false;
  double baLEDR = LEDValues.teamR;
  double baLEDB = LEDValues.teamB;
  double baLEDG = LEDValues.teamG;

  double bootDelay = 60;

  Timer preteleTimer = new Timer();



  //These are the actual preset animations.\
  //My system is bad, I reccomend learning LED stuff via the docs https://team2168.org/javadoc/com/ctre/phoenix/led/CANdle.html
  SingleFadeAnimation redFadeAnim;
  TwinkleAnimation twinkleAnim;
  //speed = 0.2;
  RainbowAnimation rainbow = new RainbowAnimation(255, 0.2, 999);


  public LEDSubsystem() {
    setLEDs(255, 0, 0);
    redFadeAnim = new SingleFadeAnimation(LEDValues.teamR, LEDValues.teamG, LEDValues.teamB);
    redFadeAnim.setSpeed(0.5);
    redFadeAnim.setNumLed(numLED);
    twinkleAnim = new TwinkleAnimation(12, 250, 140);
    twinkleAnim.setNumLed(numLED);
    twinkleAnim.setSpeed(0.05);
    SmartDashboard.putString("Battery Status", "Good!");
    preteleTimer.start();
    
  }


  @Override
   public void periodic() {
    double batteryVoltage = PowerJNI.getVinVoltage();
    if(batteryVoltage < 7) //That one thing that sees the drop in battery voltage and stops LEDs if low uwu
    {
      RobotContainer.forceLEDoff = true;
      setLEDs(0, 0, 0);
      SmartDashboard.putString("Battery Status", "NOT GOOD!");
      //SmartDashboard.putBoolean("Battery Warning", true);
      //System.out.println("AHHH!!! BATTERY DIPPING!!!!");
    }

    if(Robot.runBootAnimation)
    {
      if(bootDelay < 0)
      {
        if(baLEDLeft < baLEDRight) //Starts filling the LEDs from left end to right and vice versa, but when crossed we count it as filled in.
        {
          LED.setLEDs((int)baLEDR, (int)baLEDG, (int)baLEDB, 0, baLEDLeft, 1);
          LED.setLEDs((int)baLEDR, (int)baLEDG, (int)baLEDB, 0, baLEDRight, 1);
          baLEDLeft += 0.015;
          baLEDRight -= 0.015;
          //System.out.println("left: " + Double.toString(baLEDLeft) + ", right: " + Double.toString(baLEDRight));
        }
        else
        {
          if(!baCalledRedFade) //Once filled in, we dim all the lights to mimic the fade animation.
          {
            baLEDR = MathUtil.interpolate(baLEDR, 0, 0.06); //interpolate smoothly interpolates the two inputs.
            baLEDG = MathUtil.interpolate(baLEDG, 0, 0.06);
            baLEDB = MathUtil.interpolate(baLEDB, 0, 0.06);
            baLEDR = MathUtil.clamp(baLEDR, 0, 255); //We clamp the values between the lowest and highest possible value.
            baLEDB = MathUtil.clamp(baLEDB, 0, 255);
            baLEDG = MathUtil.clamp(baLEDG, 0, 255);
  
            setLEDs((int)baLEDR, (int)baLEDG, (int)baLEDB); //We set the values

            //setLEDs() only accepts ints (full values) so we have to convert our values over to ints.

            if(baLEDR+baLEDG+baLEDB <= 1) 
            {
              animRedFade(); //Once done we run the red fade animation.
              baCalledRedFade = true;
            }
          }
        }
        if(preteleTimer.get() > 60) //If a minute passes while waiiting pre-teleop, shutoff lights to save batteries
        {
          //setOff();
        }
      }
      else
      {
        bootDelay -= 1; //Im forcing the LEDs off to counteract the LEDs defaulting to on from other subsystems. (Im too lazy to fix it.)
        setOff();
      }
      
    }
    if(RobotContainer.forceLEDoff) 
    {
      setLEDs(0, 0, 0);
    }
  } 

  public void setLEDs(int R, int B, int G) 
  {
    clearAnimation();
    LED.setLEDs(R, G, B, 255, 0  , numLED);
    //int ledRestLength = numLED - 210;
    //LED.setLEDs(R, G, B, 255, numLED-10, 10);
  }

  public void setErrors() //UNUSED: Meant to  display errors on the LED strip. 
  {
    //LED.setLEDs(0, 255, 255, 255, numLED-10, 10);
    //We dont use this yet D:
  }

  public void animRedFade()
  {
    if(!RobotContainer.forceLEDoff)
    {
      setRainbow();
      //clearAnimation();
      //LED.animate(redFadeAnim);
      
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
      setRainbow();
      //clearAnimation();
      //LED.setLEDs(255, 0, 0);
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
      //clearAnimation();
      //LED.setLEDs(100, 0, 0);
      setRainbow();
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
