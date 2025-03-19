// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj2.command.Command;

public class auto_waitIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralSubsystem m_coral;
  private final ElevatorSubsystem m_elevator;

  double startTime; //TL;DR timers suck. (They are useful just not here)
  boolean finished = false;
  boolean resetTimer = true;
  int t = 0;
  
  //int timesRan = 1;
  //double timer = 0;
  double timeToKill = 4; //Time until to give up on the intake
  boolean pieceIsIn = false;
  boolean postPieceIn = false;

  //Timer time;
 // double timer = 0;

  public auto_waitIntake(CoralSubsystem coral, ElevatorSubsystem elevator) {
    m_coral = coral;
    m_elevator = elevator;
    //timeToKill = time;

    addRequirements(coral);
  }

  @Override
  public void initialize() {
    System.out.println("AUTO INTAKE: START THE INTAKE!!");
    pieceIsIn = false  ;
    postPieceIn = false;
    this.t = 0;
    //startTime = DriverStation.getMatchTime(); //Reset the timer
  }

  @Override
  public void execute() {
    System.out.println("AUTO INTAKE: RANGE? :" + m_coral.getRange() + "\n");
    
    if(resetTimer)
    {
      resetTimer = false; 
      pieceIsIn = false;
      postPieceIn = false;
      finished = false;

      System.out.println("AUTO INTAKE: RESET!!");
      
    }

    if(!pieceIsIn)
    {
      if((m_coral.getRange() < CoralConstants.coralInRange)) //if we grab a piece (or we pass our time to kill by a weeee bit)
      {            
        System.out.println("AUTO INTAKE: STAGE 1 DONE");   
        if(m_elevator.getRange() < ElevatorConstants.coralRange)  
        {
          pieceIsIn = true; //say we done :3
        }
      }
      else
      {
        m_coral.setSpeed(0.5);
      }
    }
  
    if((pieceIsIn) && (!postPieceIn))
    {
      m_coral.setSpeed(0.3);
      if((m_coral.getRange() > CoralConstants.coralInRange))
      {
        System.out.println("AUTO INTAKE: STAGE 2 DONE!");
        t++;
        if(t > 40)
        {
          m_coral.setSpeed(-0.05);
          postPieceIn = true;
        }
        
        
      }
    }
    if(postPieceIn)
    {
      if((m_coral.getRange() < CoralConstants.coralInRange)) //If we HAVE that piece
      {
        m_coral.setSpeed(0);
        resetTimer = true;
        System.out.println("AUTO INTAKE: IM DONE!");
        System.out.println("AUTO INTAKE: HERE IS SOME DATA:");
        System.out.println("CURRENT RANGE: " + m_coral.getRange());
        System.out.println("TARGET RANGE" + CoralConstants.coralInRange);
        System.out.println("------");
        finished = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    resetTimer = true;
  }

  @Override
  public boolean isFinished() {
    return finished; 
  }
}