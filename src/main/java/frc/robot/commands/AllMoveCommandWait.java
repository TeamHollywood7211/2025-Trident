// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AllMoveCommandWait extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_elevator;
  private final CoralSubsystem m_coral;
  
  double coralPos;
  double elevatorPos;
  double algaePos;
  boolean finished = false;
  AllMoveCommand allMoveCommand;
  private Timer waitTimer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AllMoveCommandWait(ElevatorSubsystem elevator, CoralSubsystem coral, double corPos, double ElPos, double alPos, double time) {
    m_elevator = elevator;
    m_coral = coral;
    coralPos = corPos;
    elevatorPos = ElPos;
    algaePos = alPos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("wait move started!");
    this.waitTimer = new Timer();
    this.waitTimer.start();
    this.finished = false;
    this.allMoveCommand = new AllMoveCommand(m_elevator, m_coral, coralPos, elevatorPos, algaePos);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(waitTimer.hasElapsed(1))
    {
      allMoveCommand.schedule();
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return
    return finished; 
  }
}
