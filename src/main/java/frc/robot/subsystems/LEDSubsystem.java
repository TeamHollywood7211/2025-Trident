// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDConstants;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  //CANdle LED = new CANdle(LEDConstants.ID);
  CANdle LED = new CANdle(LEDConstants.ID, RobotContainer.MainBus.getName());


  RainbowAnimation rainbow = new RainbowAnimation(255, 0.2, 999);


  public LEDSubsystem() {
    LED.setLEDs(255, 0, 0);
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
  public void setRed()
  {
    clearAnimation();
    LED.setLEDs(255, 0, 0);
  }
  public void setGreen()
  {
    clearAnimation();
    LED.setLEDs(0, 255, 0);
  }
  public void setPurple()
  {
    clearAnimation();
    LED.setLEDs(255, 0, 255);
  }
  public void setTeal()
  {
    clearAnimation();
    LED.setLEDs(0, 255, 100);
  }
  public void setDarkRed()
  {
    clearAnimation();
    LED.setLEDs(100, 0, 0);
  }
  public void setRainbow()
  {
    LED.animate(rainbow);
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
