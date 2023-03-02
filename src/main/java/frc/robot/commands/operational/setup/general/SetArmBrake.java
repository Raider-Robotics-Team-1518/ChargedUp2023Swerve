package frc.robot.commands.operational.setup.general;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/* 
 * Sets every motor involved with the arm to brake mode (excluding Telescope)
 */

public class SetArmBrake extends CommandBase {
    public SetArmBrake() {
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        RobotContainer.armSubsystem.setArmBrake();
    }
  
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
  
    @Override
    public boolean isFinished() {
        return false;
    }
}
