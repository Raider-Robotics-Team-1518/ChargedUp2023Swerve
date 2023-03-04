package frc.robot.commands.operational.claw;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/* 
* Open and close the claw at a custom set speed 
*/
public class ClawMove extends CommandBase {
    double speed = 0.25d;
    public ClawMove(double speed) {
        this.speed = speed;
        addRequirements(RobotContainer.clawSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        RobotContainer.clawSubsystem.getClawMotor().set(ControlMode.PercentOutput, speed);
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
        return true;
    }
}
