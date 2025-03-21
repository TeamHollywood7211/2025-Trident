// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  public static boolean runBootAnimation = false;

  private final boolean kUseLimelight = false;

  
  public Robot() {
    m_robotContainer = new RobotContainer();
    RobotContainer.m_LedSubsystem.setOff();
    runBootAnimation = true;
  }

  


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

   
    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    if (kUseLimelight) {

      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      



      if (llMeasurement != null && llMeasurement.tagCount > 0) {
        boolean rejectUpdate = false;
        if (llMeasurement.tagCount == 1 && llMeasurement.rawFiducials.length == 1) {
          if (llMeasurement.rawFiducials[0].ambiguity > 0.7 || llMeasurement.rawFiducials[0].distToCamera > 3) {
            rejectUpdate = true;
          }
        }
        if (!rejectUpdate) {
          m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
          m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
        }
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //m_robotContainer.m_LedSubsystem.setRed();
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void teleopExit() {
    RobotContainer.m_LedSubsystem.setTwinkle();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    
  }

  @Override
  public void simulationPeriodic() {}
}
