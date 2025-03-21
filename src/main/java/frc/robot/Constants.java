// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean safetyBypass = false; //Bypass all the safety functions


  public static class ImportantConstants {
    public static final double driveSpeed = 1.00; //In the event we need to run the robot at a slower speed
  }

//\\//\\//\\//\\//\\

  public static class AlgaeConstants {

    public static final int intakeID = 52;
    public static final int wristID = 53;
    public static final int algaeSwitch = 9;
  
    public static final double intakeSpeed = 0.5;


    public static class positions {
      public static final double home = 0.2;
      ;//0.66;

      public static final double grabbing = 0.42;//0.81;//-14; 
      public static final double grabbingHome = 0.25;//0.691;//-10;
      public static final double lowpos = 0.22;
      //public static final int home = 0.66;

      public static final int safety = 2; //TODO: Grab better safety positions :3
                                //Positive values bypass the safety
    
      public static final double top = 0.2;
      public static final double bottomL1 = 0.31; //The lowest value if we are below L1 (maybe change to L2)
      public static final double bottomPostL1 = 0.42; //The lowest value after L1 (read above for maybe)


    }


  }

  public static class LEDConstants{
    public static final int ID = 49;
  }

  public static class CoralConstants {
    public static final int intakeID = 50;
    public static final int moverID = 51;
    public static final int canRangeID = 48; 
    public static final double coralInRange = 0.106; //The value (meters) of the MAX distance between the range sensor and the game piece when typically inputted

    public static final double intakeSpeed = 0.5;


    public static class positions { //the movers positions
      public static final int right = -17;
      public static final int left = 17; //17
      public static final int home = 0;
    }
  }


  public static class autoConfigConstants {
    public static final double maxVelocityMPS = 3;
    public static final double maxVelocityMPSSq = 3;
    public static final double maxAngularVelocity = 540;
    public static final double maxAngularVelocitySq = 720;
  }
  public static class ClimberConstants {
    public static final int armMotorID = 41;
    public static final int servoOpen = 75; //The degrees to open the servo to
    public static final int clOpenPosition = -206;
    public static final int climberClose = 0;

    public static final int leftSwitchID = 7;
    public static final int rightSwitchID = 8;
  
  }
  public static class ElevatorConstants {
    public static final int leftMotorID = 42;
    public static final int rightMotorID = 43;
    public static final int canRangeID = 47;
    
    
    public static final double motorSpeed = 0.25;

    public static final double coralRange = 0.20; //TODO: Fill in


    public static class positions {
      public static final int a_barge =        210; //this is barge
      public static final int a_high =         102;
      public static final int a_low =         63; //
      public static final int a_floor =       0;
      public static final int a_processing =  31;
      public static final int a_home = 0;





      public static final int max =           120;

      public static final int c_home = 0;
      public static final int c_bottom = 11; //THESE ARE TEMP
      public static final int c_low = 28;
      public static final int c_mid = 63;
      public static final int c_high = 117;

      
      public static final int home = 0;
    }
    

  }


  public static class autoAlign{
    public static final double X_REEF_ALIGNMENT_P = 3.3;
    public static final double X_SETPOINT_REEF_ALIGNMENT = -0.34;
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
    
    
    
    public static final double Y_REEF_ALIGNMENT_P = 3.3;
    public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16;
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;



    public static final double ROT_REEF_ALIGNMENT_P = 0.058;
    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;

    public static final double DONT_SEE_TAG_TIMER = 0.5;
    public static final double POSE_VALIDATION_TIME = 0.5;
    


  }
  public static class LEDValues {
    public static final int numLED = 308;
    public static final int teamR = 255;
    public static final int teamG = 0;
    public static final int teamB = 0;
  }

  public static class ImportantPositions { //Holds all our important positions for pathplanner
    
    /*    Awesome graph detailing positions😊😊😊 (woah I can paste emojis)
     *    ________________.
     *   / LHPL           |
     *  /LHPR             |
     * |                  |
     * |     6 /  \ 5     |
     *>|    1 | ## | 4    |
     * |      2\  / 3     |
     * |                  |
     *  \ RHPL            |
         \_RHPR___________|
     We name this as if we were looking from the driver station
     The stupid numbering scheme is because of how FRC labeled the sides

     
     */
    
    
   
    public static double[] coral6 = {3.860, 5.122, 124.563  };
    public static double[] coral5 = {5.203, 5.170, 62       };
    public static double[] coral4 = {5.754, 4.055, 0        };
    public static double[] coral3 = {5, 2.9, -61.526        };
    public static double[] coral2 = {3.824, 2.952, -118.276 };
    public static double[] coral1 = {3.225, 3.959, 180      };
    //TODO: Get better name for these
    //TODO: Woahhhh the todo text has blue underlines this is SICK
    public static double[] leftHumanPlayerLeft   = {0.551, 1.382, 57.724        };
    public static double[] leftHumanPlayerRight  = {1.69, 0.591, 57.724        };
    public static double[] rightHumanPlayerLeft  = {0.659, 6.728, 162. -49.399 };
    public static double[] rightHumanPlayerRight = {1.714, 7.483, -49.399     };

  }




}
