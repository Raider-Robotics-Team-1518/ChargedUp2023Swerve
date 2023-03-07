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
    public static final double ARM_SHOULDER_UPPERSWITCH_DEG = 72d; // Degrees offset from the lower switch to the upper switch
                                                                   // with the lower switch pos being considered 0 degrees and 0 encoder position
    public static final double ARM_SHOULDER_LOWERSWITCH_DEG = 22d; // Degrees offset from the tower to the lower switch

    public static final int ARM_WRIST_ID = 32; // Neo 550

    public static final int ARM_TELESCOPE_ID = 31; // Neo 550

    public static final int ARM_CLAW_ID = 11; // HD Hex Motor

    /*
     * Static Motor Set Speeds
     */

     /* Teleoperated */
     public static final double telescopeSpeed = 0.5d;
     public static final double shoulderOffsetSpeed = 0.5d;
     public static final double wristSpeed = 0.25d;
     public static final double clawSpeed = 1d;

     /* Autonomous */
     public static final double autoTelescopeSpeed = 0.5d;
     public static final double autoClawSpeed = 0.25d;
     public static final double autoWristSpeed = 0.25d;

     public static final double clawFeedSpeed = 0.0625d;




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
     public static final double ARM_SHOULDER_P = 0.05;
     public static final double ARM_SHOULDER_I = 0.00;
     public static final double ARM_SHOULDER_D = 0.0005;

     // Wrist PID
     public static final double ARM_WRIST_P = 0.05;
     public static final double ARM_WRIST_I = 0.00;
     public static final double ARM_WRIST_D = 0.0005;

     // Telscope PID
     public static final double ARM_TELESCOPE_P = 0.05;
     public static final double ARM_TELESCOPE_I = 0.00;
     public static final double ARM_TELESCOPE_D = 0.0005;
     public static double ARM_TELESCOPE_GRAVITY_FACTOR = 0.001; // Motor Output = PIDOut + (cosine(angle in deg of arm) * ARM_TELESCOPE_GRAVITY_FACTOR)


     /* Path Planner Constants */

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
    public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(.314325,.314325);//These are in meters
    public static final Translation2d REAR_LEFT_POSITION = new Translation2d(-.314325,.314325);
    public static final Translation2d REAR_RIGHT_POSITION = new Translation2d(-.314325,-.314325);
    public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(.314325,-.314325); 

    /* Swerve Module Drive Motor Constants */
    public static final double DRIVE_ENC_TO_METERS_FACTOR = .0006343;//(1motorRev/4096 u) * (8.14 outputRev/ 1 motorRev) * ((0.1016m * 3.1415)/ 1 outputRev)
    public static final double MINIMUM_DRIVE_SPEED = 0.01;// the slowest the wheels can turn, in m/s
    public static final double MINIMUM_DRIVE_DUTY_CYCLE = 0.05;// the slowest the wheels can turn, in duty cycle output
    public static final double MOTOR_MAXIMUM_VELOCITY = 4.62; // 4.62 default
    public static final double PATH_MAXIMUM_VELOCITY = 3.5d;
    public static final double MAXIMUM_ACCELERATION = 1.25d;

    public static final double MAXIMUM_VOLTAGE = 12.0;//this is used in compensating for drops in battery voltage

    /* Swerve Move Wheel PIDF constants */ //TODO:Tune these
    public static final double SWERVE_DRIVE_P_VALUE = 0.0; 
    public static final double SWERVE_DRIVE_I_VALUE = 0.0;
    public static final double SWERVE_DRIVE_D_VALUE = 0.0;
    public static final double SWERVE_DRIVE_FF_VALUE = 1023 / (MOTOR_MAXIMUM_VELOCITY / DRIVE_ENC_TO_METERS_FACTOR);

    /* Swerve Module Rotation constants */ 
    public static final double RAD_TO_ENC_CONV_FACTOR = 8344.5488;// (1outputRev/(2*3.1415 radians)) * (12.8 motorRev / 1 outputRev) * (4096 u / 1 motorRev)
    /* PID Constants for rotation of the swerve module */
    public static final double SWERVE_ROT_P_VALUE = 0.1;
    public static final double SWERVE_ROT_I_VALUE = 0.0;
    public static final double SWERVE_ROT_D_VALUE = 0.05;
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
    public static final double DRIVER_SPEED_SCALE_LINEAR = 0.65;
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
