package frc.robot.commands.operational.setup.wrist;

import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class WristSetMaxMin extends CommandBase {
    /* Currently unusable, written due to a miscommunication of where limit switches would be 
     * This command will work for setting up the wrist when limit switches are added to the wrist
     * !!!! WILL BE MORE ACCURATE + ALLOWS FOR WRIST TO GO BELOW LEVEL (FACING STRAIGHT OUT) POSITION !!!!!
    */
    private boolean maxFound = false;
    private boolean minFound = false;

    public WristSetMaxMin() {
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        if(!minFound) {
            RobotContainer.armSubsystem.getWristMotor().set(-0.5d);
            if(RobotContainer.armSubsystem.getWristMotor().getReverseLimitSwitch(Type.kNormallyOpen).isPressed()) {
                RobotContainer.armSubsystem.resetWristPosition();
                Constants.updateDouble(Constants.WRIST_MIN_POS, RobotContainer.armSubsystem.getWristPosition());
                minFound = true;
            }
        }
        if(!maxFound && minFound) {
            RobotContainer.armSubsystem.getWristMotor().set(0.5d);
            if(RobotContainer.armSubsystem.getWristMotor().getForwardLimitSwitch(Type.kNormallyOpen).isPressed()) {
                Constants.updateDouble(Constants.WRIST_MAX_POS, RobotContainer.armSubsystem.getWristPosition());
                maxFound = true;
            }
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
        if(maxFound && minFound) {
            maxFound = minFound = false;
            return true;
        }
        return false;
    }
}
