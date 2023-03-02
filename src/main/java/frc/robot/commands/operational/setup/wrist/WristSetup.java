package frc.robot.commands.operational.setup.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;

/*
 * Toggle the command that is active in the wrist command chooser box displayed through SmartDashboard
 */
public class WristSetup extends CommandBase {
    public WristSetup() {
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        CommandScheduler.getInstance().schedule(RobotContainer.setupWristChooser.getSelected());
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
