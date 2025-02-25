// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import frc.robot.Constants;
import frc.robot.subsystems.CoralSubsystem;


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
  }

  @Override
  public void execute() {

    
    if(!pieceIsIn)
    {
      if((m_coral.getRange() < Constants.CoralConstants.coralInRange)) 
      { 
        pieceIsIn = true; 
      }
      else
      {
        m_coral.setSpeed(-0.4);
      }
    }
    if((pieceIsIn) && (!postPieceIn))
    {
      m_coral.setSpeed(-0.2);
      if(m_coral.getRange() > Constants.CoralConstants.coralInRange); //on piece exiting
      {
        postPieceIn = true;
      }
    }
    if(postPieceIn)
    {
      if(!postpostPieceIn)
      {
        if(m_coral.getRange() < Constants.CoralConstants.coralInRange)
        {
          m_coral.setSpeed(0.2);
          //postpostPieceIn = true;
          
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