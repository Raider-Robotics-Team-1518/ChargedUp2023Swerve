package frc.robot.commands.operational.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoShoulderMoveOffset extends CommandBase {
    public double opp = 0;
    public AutoShoulderMoveOffset(double offset) {
        this.opp = offset;
        addRequirements(RobotContainer.armSubsystem);
    }

    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        RobotContainer.armSubsystem.offsetShoulder(opp);
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

