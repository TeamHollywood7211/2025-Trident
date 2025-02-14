// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class CoralSubsystem extends SubsystemBase {

  TalonFX IntakeMotor1 = new TalonFX(Constants.CoralConstants.intakeID, "main");

  //RelativeEncoder IntakeEncoder = IntakeMotor1.getEncoder();
  
  //RelativeEncoder IntakeEncoder2 = IntakeMotor2.getEncoder();


 
  //DifferentialDrive differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

  public CoralSubsystem(){ 


   
    
   
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {

    return runOnce(
        () -> {

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

public void runCoralTake(double speed){
  IntakeMotor1.set(-1 * speed);
}
public void runCoralThrow(double speed){
  IntakeMotor1.set(speed);
}
public void stopCoral(){
  IntakeMotor1.set(0);
}

}