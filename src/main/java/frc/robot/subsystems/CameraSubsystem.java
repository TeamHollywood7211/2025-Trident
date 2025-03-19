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

  UsbCamera camera1; //front camera
  UsbCamera camera2; //backup camera
  VideoSink server;
  boolean toggleCam = false;

  public CameraSubsystem() {

    camera1 = CameraServer.startAutomaticCapture("Coral Cam", 0);
    camera2 = CameraServer.startAutomaticCapture("Climber Cam", 1);
    
    camera1.setResolution(70, 70);
    camera1.setFPS(15);
    camera2.setResolution(50, 50);
    camera2.setFPS(10);
    server = CameraServer.getServer();
    server.setSource(camera1);


  }



  public void toggleCam()
  {
    int i = 0;
    System.out.println("Initiating camera switch, this may need a few attempts!");
    toggleCam = !toggleCam;
    if(toggleCam) //Sometimes it takes a second switch to get the camera actually working. (Kinda noticed I think my order of operations was just effed, oops!)
    {
      System.out.println("Switching to CAM 1: Closing CAM 2...") ;
      while(i < 4)
      {
        camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen)  ;
        camera2.setConnectionStrategy(ConnectionStrategy.kForceClose);
        server.setSource(camera1);
        i++;
      }
      }
    else{
      System.out.println("Switching to CAM 2: Closing CAM 1...") ;
      while(i < 4)
      {
        
        
        camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen)  ; //Changes camera connection strategy.
        camera1.setConnectionStrategy(ConnectionStrategy.kForceClose);
        server.setSource(camera2);
        i++;
      }

      
    }
  }
}
