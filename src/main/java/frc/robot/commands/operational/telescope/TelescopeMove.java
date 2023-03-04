package frc.robot.commands.operational.telescope;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/*
 * Sets the telescope motor to a static positive speed.
 */
public class TelescopeMove extends CommandBase {
    double speed = 0.25d;
    public TelescopeMove(double speed) {
        this.speed = speed;
        addRequirements(RobotContainer.armSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        //if(RobotContainer.armSubsystem.isTelescopeInRange()) {
            RobotContainer.armSubsystem.setTelescopeSpeed(speed);
        //}
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
