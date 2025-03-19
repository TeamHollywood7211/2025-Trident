// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class waitIntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralSubsystem m_coral;

  Double startTime; //TL;DR timers suck. (They are useful just not here)
  boolean finished = false;
  boolean resetTimer = true;
  double timer = 0;
  double timeToKill = 4; //Time until to give up on the intake
  boolean pieceIsIn = false;
  boolean postPieceIn = false;
  boolean postpostPieceIn = false;
  Timer intakeTimer;
  int t = 0;

  //Timer time;
 // double timer = 0;

  public waitIntakeCommand(CoralSubsystem coral) {
    m_coral = coral;
    //timeToKill = time;

    addRequirements(coral);
  }

  @Override
  public void initialize() {
    pieceIsIn = false;
    postPieceIn = false;
    postpostPieceIn = false;
    finished = false;
    this.t = 0;
    this.intakeTimer.reset();
    this.intakeTimer.start();
  }

  @Override
  public void execute() {

    
    //This code is for the intake mechanism, we cannot trust ourselves to safetly intake a piece
    //and so we utilize a distance sensor to automatically intake it. We first
    /*
     * 1. Run the intake until the distance sensor is tripped and a set time has passed, then untripped.
     * 2. Run the intake backwards until the sensor is tripped.
     * 3. Return true on isFinished();
     * 
     * The set time was added to give us a bit of delay in the event that the sensor was skidishly tripped.
     * 
     */



    if(!pieceIsIn) //Sensor not passed first trip
    {
      if((m_coral.getRange() < Constants.CoralConstants.coralInRange))
      {
        System.out.println("****INTAKE: STEP 1****");
        pieceIsIn = true; //If the sensor is tripped, say we passed the first check
      }
      else
      {
        m_coral.setSpeed(-0.4); //if not tripped, run the intake and run again
        intakeTimer.reset();
      }
    }
    if((pieceIsIn) && (!postPieceIn)) //if pass first check but not second
    {
      m_coral.setSpeed(-0.2); //Slow the speed down to not overshoot
      if((m_coral.getRange() > Constants.CoralConstants.coralInRange) && (intakeTimer.get() > 0.5)); //on piece exiting
      { //If the distance sensor doesnt see the piece, and the timer has passed
        postPieceIn = true;//say second check done
        System.out.println("****INTAKE: STEP 2****");
      }
    }
    if(postPieceIn) //after second check
    {
      if(!postpostPieceIn) //if the THIRD check isnt passed 
      {
        if(m_coral.getRange() < Constants.CoralConstants.coralInRange)
        {

          m_coral.setSpeed(0.2); //Run the intake backwards
          System.out.println("****INTAKE: STEP 3****");
          postpostPieceIn = true;
          
        }
        else{
          postpostPieceIn = false;
          postPieceIn = false;
          pieceIsIn = false;
          finished = true;
        }
      }
    }
    
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return finished; 
  }
}