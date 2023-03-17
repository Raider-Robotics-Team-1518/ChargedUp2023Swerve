// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Preferences;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
import edu.wpi.first.wpilibj2.command.Command;
public final class Constants {
    /*
     * Raider Robotics Team 1518
     */

    public static boolean setupState = false;

    public static final int ARM_SHOULDER_ID = 30; // Normal Neo
    //public static final double ARM_SHOULDER_LEVEL_DEG = 69d; // Degrees offset from the lower switch with the lower switch pos being considered 0 degrees and 0 encoder position
    public static final double ARM_SHOULDER_UPPERSWITCH_DEG = 74d; // Degrees offset from the lower switch to the upper switch
                                                                   // with the lower switch pos being considered 0 degrees and 0 encoder position
    public static final double ARM_SHOULDER_LOWERSWITCH_DEG = 24d; // Degrees offset from the tower to the lower switch

    public static final int ARM_WRIST_ID = 666; // Neo 550

    public static final int ARM_TELESCOPE_ID = 31; // Neo 550

    public static final int ARM_CLAW_ID1 = 33; // left

    public static final int ARM_CLAW_ID2 = 32; // right

    /*
     * Static Motor Set Speeds
     */

     /* Teleoperated */
     public static final double telescopeSpeed = 1.0d;
     public static final double shoulderUpSpeed = 0.425d;
     public static final double shoulderDownSpeed = 0.2125d;
     public static final double wristSpeed = 0.25d;
     public static final double clawFeedSpeed = 0.375d;
     public static final double clawDropSpeed = 0.5d;

     /* Autonomous */
     public static final double autoTelescopeSpeed = 0.5d;
     public static final double autoClawSpeed = 0.25d;
     public static final double autoWristSpeed = 0.25d;




    /*
     * Preference Names
     */

     /* Shoulder */
     public static final String SHOULDER_IDLE_ANGLE = "shoulderIdleAngle";
     public static final String SHOULDER_MAX_POS = "shoulderMaxPos";
     public static final String SHOULDER_LEVEL_POS = "shoulderLevelPos";

     /* Wrist */
     public static final String WRIST_IDLE_ANGLE = "wristIdleAngle";
     public static final String WRIST_MAX_POS = "wristMaxPos";
     public static final String WRIST_MIN_POS = "wristMinPos";

     /* Telescope */
     public static final String TELESCOPE_MAX_POS = "telescopeMaxPos";
     public static final String TELESCOPE_MIN_POS = "telescopeMinPos";

    /*
     * Non-Swerve PID
     */
     // Shoulder PID
     public static final double ARM_SHOULDER_P = 0.05; // 0.05
     public static final double ARM_SHOULDER_I = 0.00; 
     public static final double ARM_SHOULDER_D = 0.0005; // 0.0005

     // Wrist PID
     public static final double ARM_WRIST_P = 0.05;
     public static final double ARM_WRIST_I = 0.00;
     public static final double ARM_WRIST_D = 0.0005;

     // Telscope PID
     public static final double ARM_TELESCOPE_P = 0.05;
     public static final double ARM_TELESCOPE_I = 0.00;
     public static final double ARM_TELESCOPE_D = 0.0005;
     public static double ARM_TELESCOPE_GRAVITY_FACTOR = 0.001; // Motor Output = PIDOut + (cosine(angle in deg of arm) * ARM_TELESCOPE_GRAVITY_FACTOR)
     public static final double ARM_TELESCOPE_VELOCITY = 0.25d; // Measured in meters per second TODO

     /* Field Constants */

     /* Center of Field Offsets */
     public static final double FIELD_CENTER_X = 8.270875d;
     public static final double FIELD_CENTER_Y = 4.00685d;

     // Measured in Meters
     public static final double DIST_TO_UPPER_NODE_CONE = 1.01d;
     public static final double DIST_TO_MIDDLE_NODE_CONE = 0.58d;

     public static final double DIST_TO_UPPER_NODE_CUBE = 1.01d;
     public static final double DIST_TO_MIDDLE_NODE_CUBE = 0.58d;

     public static final double DIST_TO_LOWER_NODE = 0.29d;


     /* Robot Dimensions */
     public static final double ROBOT_SIZE = 0.78105d;
     public static final double TOWER_HEIGHT_TO_PIVOT = 1.1684; // 46 inches
     public static final double TELESCOPE_LENGTH_RETRACTED = 0.9144;


     

     

     /* Path Planner Event Map */
     public static HashMap<String, Command> autonomousEventMap = new HashMap<>();
     

     /*
      * Greater Rochester Robotics
      */
    /* Factors of PI */
    public static final double PI_OVER_TWO = Math.PI/2;
    public static final double THREE_PI_OVER_TWO = 3*PI_OVER_TWO;
    public static final double TWO_PI = 2*Math.PI;
    public static final Rotation2d ROTATE_BY_PI = Rotation2d.fromDegrees(180);

    /* Swerve Module Positions */
    public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(0.4826,0.4826);//These are in meters
    public static final Translation2d REAR_LEFT_POSITION = new Translation2d(-0.4826,0.4826);
    public static final Translation2d REAR_RIGHT_POSITION = new Translation2d(-0.4826,-0.4826);
    public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(0.4826,-0.4826); 

    /* Swerve Module Drive Motor Constants */
    public static final double DRIVE_ENC_TO_METERS_FACTOR = 0.00003829298477;
    // rob old: (1motorRev/4096 u) * (8.14 outputRev/ 1 motorRev) * ((0.1016m * 3.1415)/ 1 outputRev) = 0.0006343
    // semi working value (tested): 0.0000382966
    // !! newest calculation (not tested) !!
    // encoderPos*DRIVE_ENC_TO_METERS_FACTOR=distanceTraveled
    // encoderPos*(1/2048)*(1/8.14)*(2*3.14159265*0.1016)=distanceTraveled
    // (1/2048)*(1/8.14)*(2*3.14159265*0.1016) = 0.00003829298477 
    // (Calculated using Full Precision calculator, up to 100 decimal places)

    public static final double MINIMUM_DRIVE_SPEED = 0.01;// the slowest the wheels can turn, in m/s
    public static final double MINIMUM_DRIVE_DUTY_CYCLE = 0.05;// the slowest the wheels can turn, in duty cycle output
    public static final double MOTOR_MAXIMUM_VELOCITY = 4.1; // 4.62 default
    public static final double PATH_MAXIMUM_VELOCITY = 2.75d;
    public static final double MAXIMUM_ACCELERATION = 1.25d;
    public static final double PATH_MAXIMUM_ACCELERATION = 1.25;

    public static final double MAXIMUM_VOLTAGE = 12.0;//this is used in compensating for drops in battery voltage

    /* Swerve Move Wheel PIDF constants */
    /* PID Constants for rotation of the swerve module */
    /* Input: ControlPercent Speed
     * Output: Translation Position (Pose2d X or Y)
     */
    public static final double SWERVE_DRIVE_P_VALUE = 0.2928; //0.2928
    public static final double SWERVE_DRIVE_I_VALUE = 0.0;
    public static final double SWERVE_DRIVE_D_VALUE = 0.007322; // 0.00089375
    public static final double SWERVE_DRIVE_FF_VALUE = 1023 / (MOTOR_MAXIMUM_VELOCITY / DRIVE_ENC_TO_METERS_FACTOR);
    //public static final double SWERVE_DRIVE_FF_VALUE = 0.0d;

    /* Swerve Module Rotation constants */ 
    public static final double RAD_TO_ENC_CONV_FACTOR = (512*8.14)/(Math.PI/2);
    // rob old (tested, broken): (1outputRev/(2*3.1415 radians)) * (12.8 motorRev / 1 outputRev) * (4096 u / 1 motorRev) = 8344.5488
    // !! newest calculation (tested, weird) !!
    // (1/(2*pi))*2048*8.14 = 2653.2274929
    // 512/(pi/2)=325.949323452
    // (512*8.14)/(Math.PI/2)=2653.2274929

    /* PID Constants for rotation of the swerve module */
    /* Input: Motor ControlPercent Speed
     * Output: Encoder Position
     */
    public static final double SWERVE_ROT_P_VALUE = 0.2501474825939262; // 0.02*4 rob
    public static final double SWERVE_ROT_I_VALUE = 0.0;
    public static final double SWERVE_ROT_D_VALUE = 0.005002949651878525;  // .05*4 rob
    public static final double SWERVE_ROT_I_ZONE_VALUE = 0;
    public static final double SWERVE_ROT_FF_VALUE = 0.0;
    
    public static final double SWERVE_MODULE_TOLERANCE = 0.1;
    public static final double ROTATIONAL_VELOCITY_TOLERANCE = 1.0;

    /* Robot Rotation PID controller constants */
    public static final double ROBOT_SPIN_PID_TOLERANCE = Math.toRadians(0.5);
    public static final double MINIMUM_ROTATIONAL_OUTPUT = 0.10;

    /* Constant for turn to angle functions */
    public static final double ROBOT_SPIN_P = 1.55;//tuned for drive/climber bot
    public static final double ROBOT_SPIN_I = 0.0;
    public static final double ROBOT_SPIN_D = 0.01;
    
    /* Constants to stop incidental rotation for motion */
    public static final double ROBOT_COUNTER_SPIN_P = 1.1;
    public static final double ROBOT_COUNTER_SPIN_I = 0.0;
    public static final double ROBOT_COUNTER_SPIN_D = 0.001;

    /* Driver Scaling Constants */
    public static final double DRIVER_SPEED_SCALE_LINEAR = 0.5;
    public static final double DRIVER_SPEED_SCALE_ROTATIONAL = .75;

    
    /* IDENTIFICATION NUMBERS FOR DEVICES */

    /* CTRE motor and sensors */

    public static final int FRONT_LEFT_MOVE_MOTOR = 1;//drive module 0
    public static final int FRONT_LEFT_ROTATE_MOTOR = 2;//drive module 0
    public static final int FRONT_LEFT_ROTATE_SENSOR = 15;//drive module 0

    public static final int REAR_LEFT_MOVE_MOTOR = 3;//drive module 1
    public static final int REAR_LEFT_ROTATE_MOTOR = 4;//drive module 1
    public static final int REAR_LEFT_ROTATE_SENSOR = 12;//drive module 1

    public static final int REAR_RIGHT_MOVE_MOTOR = 5;//drive module 2
    public static final int REAR_RIGHT_ROTATE_MOTOR = 6;//drive module 2
    public static final int REAR_RIGHT_ROTATE_SENSOR = 13;//drive module 2
    
    public static final int FRONT_RIGHT_MOVE_MOTOR = 7;//drive module 3
    public static final int FRONT_RIGHT_ROTATE_MOTOR = 8;//drive module 3
    public static final int FRONT_RIGHT_ROTATE_SENSOR = 14;//drive module 3

    /* Rev Robotics SparkMAXs */
    
    /* Solenoid Channels */
    
    /* Digital Input */

    public enum PlaceMode {
      LOW_NODE_CONE(ROBOT_SIZE-DIST_TO_LOWER_NODE),
      MID_NODE_CONE(ROBOT_SIZE-DIST_TO_MIDDLE_NODE_CONE),
      HIGH_NODE_CONE(ROBOT_SIZE-DIST_TO_UPPER_NODE_CONE),
      LOW_NODE_CUBE(ROBOT_SIZE-DIST_TO_LOWER_NODE),
      MID_NODE_CUBE(ROBOT_SIZE-DIST_TO_MIDDLE_NODE_CUBE),
      HIGH_NODE_CUBE(ROBOT_SIZE-DIST_TO_UPPER_NODE_CUBE);
      // Math.asin((ROBOT_SIZE-DIST_TO_UPPER_NODE_CUBE))/TOWER_HEIGHT_TO_PIVOT
      private final double distance;
      private PlaceMode(double distance) {
        this.distance = distance;
      }

      public double getAngle() {
        double resAngle = Math.asin((this.distance)/TOWER_HEIGHT_TO_PIVOT);
        resAngle = 90-Math.abs(resAngle);
        return resAngle;
      }

      public double getWaitTime() {
        double neededExtensionMeters = ((this.distance)/Math.sin(getAngle()))*Math.sin(90);
        neededExtensionMeters = Math.abs(neededExtensionMeters);
        neededExtensionMeters -= TELESCOPE_LENGTH_RETRACTED;
        return neededExtensionMeters/ARM_TELESCOPE_VELOCITY;
      }
    }

    public static void updateDouble(String key, double newValue) {
      try { Preferences.remove(key); } catch(Exception exception) {}
      Preferences.setDouble(key, newValue);
    }

    public static double getDouble(String key) {
      if(Preferences.containsKey(key)) {
        return Preferences.getDouble(key, -69);
      }
      return -69;
    }

    
}
