/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController.Axis;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;


/**
 * This command is designed so that a driver can drive 
 * the swerve drive based around a fixed orientation.
 * Forward on the stick should cause the robot to away 
 * from the driver. If this is true, then left and right 
 * on the stick will cause the robot to move to the 
 * driver's left and right, respectively. This command 
 * does not end of its own accord so it must be interupted 
 * to end.
 */
public class DriveFieldRelative extends CommandBase {
  private boolean veloMode;
  /**
   * Creates a new DriveFieldCentric.
   */
  public DriveFieldRelative(boolean veloMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
    this.veloMode = veloMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.swerveDrive.isFieldRelative = true;
    double  awaySpeed = -Robot.robotContainer.getDriverAxis(Axis.kLeftY);
    double lateralSpeed = -Robot.robotContainer.getDriverAxis(Axis.kLeftX);
    //check if secondary sticks are being used
    if(Math.abs(Robot.robotContainer.getDriverAxis(Axis.kRightY))>.1 ||
     Math.abs(Robot.robotContainer.getDriverAxis(Axis.kRightX))>.1){
     //if secondary sticks used, replace with secondary sticks witha slow factor
     awaySpeed = Robot.robotContainer.getDriverAxis(Axis.kRightY)*.375*0.5;
     lateralSpeed = Robot.robotContainer.getDriverAxis(Axis.kRightX)*.375*0.5;
    }
    double rotSpeed = Robot.robotContainer.getDriverAxis(Axis.kLeftTrigger) - Robot.robotContainer.getDriverAxis(Axis.kRightTrigger);

    RobotContainer.swerveDrive.driveFieldRelative(
      awaySpeed*Constants.DRIVER_SPEED_SCALE_LINEAR,
      lateralSpeed*Constants.DRIVER_SPEED_SCALE_LINEAR_LATERAL,
      rotSpeed*-Constants.DRIVER_SPEED_SCALE_ROTATIONAL, 
      veloMode
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
