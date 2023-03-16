package frc.robot.commands.operational.setup.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

/*
 * Toggle the command that is active in the shoulder command chooser box displayed through SmartDashboard
 */
public class ShoulderSetup extends CommandBase {
    public ShoulderSetup() {
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        CommandScheduler.getInstance().schedule(RobotContainer.setupShoulderChooser.getSelected());
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
