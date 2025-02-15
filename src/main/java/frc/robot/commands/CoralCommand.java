package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralSubsystem;
//import frc.robot.Constants.*;

public class CoralCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private CoralSubsystem m_coral;
  private CommandXboxController m_controller;

  public CoralCommand(CoralSubsystem intakeSubsystem, CommandXboxController controller) {
    this.m_controller = controller;
    this.m_coral = intakeSubsystem;
    addRequirements(intakeSubsystem);
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  
  public void execute() {
    
    m_coral.runCoral(RobotContainer.booleanToDouble(m_controller.leftBumper().getAsBoolean())
     - RobotContainer.booleanToDouble(m_controller.rightBumper().getAsBoolean()));
    //System.out.println(RobotContainer.booleanToDouble(m_controller.leftBumper().getAsBoolean())
    //- RobotContainer.booleanToDouble(m_controller.rightBumper().getAsBoolean()));
  
     if(Math.abs(m_controller.getLeftX()) > 0.05 )
     {
      m_coral.addPosition(m_controller.getLeftX() * 0.1);
     }
  
    }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}