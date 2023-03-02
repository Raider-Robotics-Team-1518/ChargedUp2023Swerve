package frc.robot.commands.operational.setup.shoulder;

import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShoulderSetMaxMin extends CommandBase {
    private boolean maxFound = false;
    private boolean minFound = false;
    private boolean calculated = false;

    double upperEncVal, lowerEncVal;

    public ShoulderSetMaxMin() {
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
        upperEncVal = lowerEncVal = 0;
        RobotContainer.armSubsystem.setArmBrake();
    }
  
    @Override
    public void execute() {
        if(!maxFound) {
            RobotContainer.armSubsystem.getShoulderMotor().set(0.25d);
            if(RobotContainer.armSubsystem.getShoulderMotor().getForwardLimitSwitch(Type.kNormallyOpen).isPressed()) {
                RobotContainer.armSubsystem.getShoulderMotor().set(0.0d);
                upperEncVal = RobotContainer.armSubsystem.getShoulderPosition();
                maxFound = true;
            }
            return;
        }
        if(maxFound && !minFound) {
            RobotContainer.armSubsystem.getShoulderMotor().set(-0.125d);
            if(RobotContainer.armSubsystem.getShoulderMotor().getReverseLimitSwitch(Type.kNormallyOpen).isPressed()) {
                RobotContainer.armSubsystem.getShoulderMotor().set(0.0d);
                lowerEncVal = RobotContainer.armSubsystem.getShoulderPosition();
                minFound = true;
            }
            return;
        }
        if(maxFound && minFound && !calculated) {
            double encoderCountObtainable = upperEncVal + Math.abs(lowerEncVal);
            double totalEncoderCounts = (1/((Constants.ARM_SHOULDER_UPPERSWITCH_DEG-Constants.ARM_SHOULDER_LOWERWITCH_DEG)/180))*encoderCountObtainable; // get amount of encoder counts are from 0 deg to 180 deg
            
            double countsAbove = 1-(Constants.ARM_SHOULDER_UPPERSWITCH_DEG/180)*totalEncoderCounts; // Get how many encoder counts are between the upper switch and 180 deg (sky)

            double countsBelow = (Constants.ARM_SHOULDER_LOWERWITCH_DEG/180)*totalEncoderCounts; // Get how many encoder counts are between the lower switch and 0 deg (floor)
            
            double maxPos = countsAbove+upperEncVal; // Offset from the encoder value we get when the upper switch is hit
            double minPos = lowerEncVal-countsBelow; // Offset from the encoder value we get when the lower switch is hit\
            Constants.updateDouble(Constants.SHOULDER_MAX_POS, maxPos);
            Constants.updateDouble(Constants.SHOULDER_MIN_POS, minPos);
            calculated = true;
        }
    }
  
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
  
    @Override
    public boolean isFinished() {
        if(calculated) {
            calculated = false;
            minFound = maxFound = false;
            upperEncVal = lowerEncVal = 0;
            return true;
        }
        return false;
    }
}
