// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import frc.robot.Constants;
import frc.robot.subsystems.CoralSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class auto_waitIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralSubsystem m_coral;

  Double startTime; //TL;DR timers suck. (They are useful just not here)
  boolean finished = false;
  boolean resetTimer = true;
  
  int timesRan = 1;
  double timer = 0;
  double timeToKill = 4; //Time until to give up on the intake
  boolean pieceIsIn = false;
  boolean postPieceIn = false;

  //Timer time;
 // double timer = 0;

  public auto_waitIntake(CoralSubsystem subsystem, double time) {
    m_coral = subsystem;
    timeToKill = time;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    System.out.println("AUTO INTAKE: START THE INTAKE!!");
    //time.reset();
    startTime = DriverStation.getMatchTime(); //Reset the timer
  }

  @Override
  public void execute() {
    
    System.out.println("AUTO INTAKE: TIME: " + (startTime - DriverStation.getMatchTime()) + "  ");
    System.out.println("AUTO INTAKE: RING? :" + m_coral.getRange() + "\n");
    m_coral.setSpeed(0.5);


    if(resetTimer)
    {
      startTime = DriverStation.getMatchTime(); //sets the timer the timer started to now
      resetTimer = false; //says "yo we resetted"
      System.out.println("AUTO INTAKE: RESETING TIMER");
      timer = 0; //sets the actual timer variable to 0
    }


    

    
    timer = (startTime - DriverStation.getMatchTime()); //calculate time between now and last time we reset timer
    //time.start();
    //timer = time.get();


    if(timer > timeToKill) //if we dont grab and we pass our time to kill
    {
      //uhhh
    }
    //(timer > timeToKill + 0.1) || 
    if((m_coral.getRange() < Constants.CoralConstants.coralInRange)) //if we grab a piece (or we pass our time to kill by a weeee bit)
    {
      
      System.out.println("AUTO INTAKE: MAY OR MAY NOT HAVE THE CORAL, IDC WE MOVIN' "); 
      System.out.println("AUTO INTAKE: TIME RAN: " + timesRan);                          
      pieceIsIn = true; //say we done :3
    }
    else
    {
      finished = false; //if not done, say we aint done
    }

    if(pieceIsIn)
    {
      m_coral.setSpeed(0.3);
      if(!(m_coral.getRange() < Constants.CoralConstants.coralInRange))
      {
        m_coral.setSpeed(-0.05);
        postPieceIn = true;
        
      }
    }
    if(postPieceIn)
    {
      if((m_coral.getRange() < Constants.CoralConstants.coralInRange))
      {
        m_coral.setSpeed(0);
        finished = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    resetTimer = true;
    timesRan++; //Just a bit of debug info for me 
  }

  @Override
  public boolean isFinished() {
    return finished; 
  }
}