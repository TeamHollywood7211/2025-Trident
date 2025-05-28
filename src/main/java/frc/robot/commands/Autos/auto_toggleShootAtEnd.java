// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class auto_toggleShootAtEnd extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralSubsystem m_subsystem;
  private boolean finished = false;
  
    public auto_toggleShootAtEnd(CoralSubsystem subsystem) {
      m_subsystem = subsystem;
      addRequirements(subsystem);
    }
    @Override
    public void initialize() {}
  
    @Override
    public void execute() {
      RobotContainer.shootAtAutoEnd = true;
      this.finished = true;
  }
  @Override
  public void end(boolean interrupted) {}
  @Override
  public boolean isFinished() {
    return this.finished;
  }
}
