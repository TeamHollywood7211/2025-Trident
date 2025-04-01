// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import frc.robot.subsystems.AlgaeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class auto_sendMessage extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeSubsystem m_algae;
  private final String m_message;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public auto_sendMessage(AlgaeSubsystem algae, String string) {
    m_algae = algae;
    m_message = string;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(m_message);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
