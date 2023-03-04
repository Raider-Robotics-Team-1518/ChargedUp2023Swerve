package frc.robot.commands.operational.setup.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShoulderMoveLevel extends CommandBase {
    public ShoulderMoveLevel() {
        addRequirements(RobotContainer.armSubsystem);
    }

    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        RobotContainer.armSubsystem.setShoulderTargetPos(90, true);
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


