package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems. CoralSubsystem;
//import frc.robot.Constants.*;

public class CoralCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private CoralSubsystem m_coralIntakeSubsystem;
  private CommandXboxController m_controller;

  public CoralCommand(CoralSubsystem intakeSubsystem, CommandXboxController controller) {
    this.m_controller = controller;
    this.m_coralIntakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_controller.leftBumper().equals(true)){
      m_coralIntakeSubsystem.runCoralTake(1);
      
      // TODO: we gotta fix the parameters for the 
    } else if(m_controller.rightBumper().equals(true)){
      m_coralIntakeSubsystem.runCoralThrow(1);
    }
    else{
      //m_gripSubsystem.setGripOut();
      m_coralIntakeSubsystem.stopCoral();
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  
  public void grabCoralHard(double speed){
    m_coralIntakeSubsystem.runCoralTake(speed * 2);
    } 
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}