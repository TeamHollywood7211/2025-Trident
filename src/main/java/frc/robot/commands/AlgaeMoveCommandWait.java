// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlgaeMoveCommandWait extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_elevator;
  private final AlgaeSubsystem m_algae;
  
  double algaePos;
  double elevatorPos;
  boolean finished = false;
  AlgaeMoveCommand algaeMoveCommand;
  private Timer waitTimer;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeMoveCommandWait(ElevatorSubsystem elevator, AlgaeSubsystem algae, double ElPos, double alPos) {
    m_elevator = elevator;
    m_algae = algae;
    elevatorPos = ElPos;
    algaePos = alPos;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.waitTimer = new Timer();
    this.waitTimer.start();
    this.finished = false;
    this.algaeMoveCommand = new AlgaeMoveCommand(m_elevator, m_algae, elevatorPos, algaePos);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(waitTimer.hasElapsed(1))
    {
      algaeMoveCommand.schedule();
      
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return algaeMoveCommand.isFinished();
  }
}
