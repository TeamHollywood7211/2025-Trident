package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
<<<<<<<< HEAD:src/main/java/frc/robot/commands/AlgaeCommand.java
import frc.robot.RobotContainer;
import frc.robot.subsystems.AlgaeSubsystem;
//import frc.robot.Constants.*;

public class AlgaeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private AlgaeSubsystem m_intakeSubsystem;
  private CommandXboxController m_controller;
  public AlgaeCommand(AlgaeSubsystem intakeSubsystem, CommandXboxController controller) {
    this.m_controller = controller;
    this.m_intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  } 
========
import frc.robot.subsystems.AlgaeIntakeSubsystem;
//import frc.robot.Constants.*;

public class AlgaeIntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private AlgaeIntakeSubsystem m_algaeIntakeSystem;
  private CommandXboxController m_controller;
>>>>>>>> 3da98093c7483fd25575c406b200538c0ce8e2a6:src/main/java/frc/robot/commands/AlgaeIntakeCommand.java

  public AlgaeIntakeCommand(AlgaeIntakeSubsystem intakeSubsystem, CommandXboxController controller) {
    this.m_controller = controller;
    this.m_algaeIntakeSystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
    
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<<< HEAD:src/main/java/frc/robot/commands/AlgaeCommand.java

    m_intakeSubsystem.runGrip(RobotContainer.booleanToDouble(m_controller.leftTrigger().getAsBoolean()) - RobotContainer.booleanToDouble(m_controller.rightTrigger().getAsBoolean()));
    //System.out.println(RobotContainer.booleanToDouble(m_controller.leftTrigger().getAsBoolean()));
  

    if(Math.abs(m_controller.getRightY()) > 0.05 )
    {
      m_intakeSubsystem.setPosition(m_controller.getRightY() * 0.1);
========
    if(m_controller.getRightTriggerAxis() > 0.05){
      m_algaeIntakeSystem.runGripIn(m_controller.getRightTriggerAxis());
      
    } else if(m_controller.getLeftTriggerAxis() > 0.05){
      m_algaeIntakeSystem.runGripOut(m_controller.getLeftTriggerAxis());

    }
    else{
      //m_gripSubsystem.setGripOut();
      m_algaeIntakeSystem.stopGrip();
>>>>>>>> 3da98093c7483fd25575c406b200538c0ce8e2a6:src/main/java/frc/robot/commands/AlgaeIntakeCommand.java
    }
    
  
  
  
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
<<<<<<<< HEAD:src/main/java/frc/robot/commands/AlgaeCommand.java

========
  
  public void grabAlgaeHard(double speed){
    m_algaeIntakeSystem.runGripIn(speed * 2);
    } 
  
>>>>>>>> 3da98093c7483fd25575c406b200538c0ce8e2a6:src/main/java/frc/robot/commands/AlgaeIntakeCommand.java
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}