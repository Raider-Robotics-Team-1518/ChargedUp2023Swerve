package frc.robot.commands.operational.util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ArmTurnToOffset extends CommandBase {
    double opp = 0;
    public ArmTurnToOffset(double offset) {
        this.opp = offset;
        addRequirements(RobotContainer.armSubsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.armSubsystem.armPidController.setSetpoint(RobotContainer.armSubsystem.desiredPoint+opp);
        RobotContainer.armSubsystem.desiredPoint = RobotContainer.armSubsystem.desiredPoint + opp;
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
  }

