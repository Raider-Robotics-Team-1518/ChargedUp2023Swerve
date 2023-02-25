package frc.robot.commands.operational.setup.shoulder;

import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShoulderSetMaxMin extends CommandBase {
    private boolean maxFound = false;
    private boolean minFound = false;
    private boolean calculated = false;

    public ShoulderSetMaxMin() {
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        // 25 deg to 155 deg

        double upperEncVal, lowerEncVal;
        upperEncVal = lowerEncVal = 0;

        if(!maxFound) {
            RobotContainer.armSubsystem.getShoulderMotor().set(0.125d);
            if(RobotContainer.armSubsystem.getShoulderMotor().getForwardLimitSwitch(Type.kNormallyOpen).isPressed()) {
                upperEncVal = RobotContainer.armSubsystem.getShoulderPosition();
                maxFound = true;
            }
        }
        if(maxFound && !minFound) {
            RobotContainer.armSubsystem.getShoulderMotor().set(-0.125d);
            if(RobotContainer.armSubsystem.shoulderLowerSwitch.get()) {
                RobotContainer.armSubsystem.getShoulderMotor().set(0.0d);
                lowerEncVal = RobotContainer.armSubsystem.getShoulderPosition();
                minFound = true;
            }
        }
        double encoderDifference = upperEncVal - lowerEncVal;
        double totalEncoderCounts = (1/((Constants.ARM_SHOULDER_UPPERSWITCH_DEG-Constants.ARM_SHOULDER_LOWERWITCH_DEG)/180))*encoderDifference; // How many encoder counts are from 0 deg to 180 deg
        double oneEightyPos = 1-(Constants.ARM_SHOULDER_UPPERSWITCH_DEG/180)*totalEncoderCounts;
        double zeroPos = (Constants.ARM_SHOULDER_LOWERWITCH_DEG/180)*totalEncoderCounts;
        Preferences.setDouble(Constants.SHOULDER_MAX_POS, oneEightyPos);
        Preferences.setDouble(Constants.SHOULDER_MIN_POS, zeroPos);
        calculated = true;
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
            return true;
        }
        return false;
    }
}
