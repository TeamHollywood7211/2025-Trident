// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  UsbCamera camera1;
  UsbCamera camera2;
  VideoSink server;
  boolean toggleCam = false;

  public CameraSubsystem() {

    camera1 = CameraServer.startAutomaticCapture("Coral Cam", 0);
    camera2 = CameraServer.startAutomaticCapture("Climber Cam", 1);
        //server = CameraServer.getServer();
        //server.setSource(camera1);
    //camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    //camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    camera1.setResolution(70, 70);
        //camera1.setPixelFormat(PixelFormat.kGray);
        //camera2.setPixelFormat(PixelFormat.kGray);
    camera1.setFPS(15);
    camera2.setResolution(50, 50);
    camera2.setFPS(10);
    server = CameraServer.getServer();
    server.setSource(camera1);


  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void toggleCam()
  {
    toggleCam = !toggleCam;
    if(toggleCam)
    {
      System.out.println("Switching to CAM 1: Closing CAM 2...");
      server.setSource(camera1);
      camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      camera2.setConnectionStrategy(ConnectionStrategy.kForceClose);
    }
    else{

      
      System.out.println("Switching to CAM 2: Closing CAM 1...");
      server.setSource(camera2);
      camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      camera1.setConnectionStrategy(ConnectionStrategy.kForceClose);
      
    }
  }
}
