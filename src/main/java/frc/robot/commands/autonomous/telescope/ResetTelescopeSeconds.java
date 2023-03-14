package frc.robot.commands.autonomous.telescope;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.autonomous.AutoCommandGroups;

public class ResetTelescopeSeconds extends CommandBase {
    public ResetTelescopeSeconds() {
        addRequirements(RobotContainer.swerveDrive, RobotContainer.armSubsystem, RobotContainer.clawSubsystem);
    }

    @Override
    public void execute() {
        AutoCommandGroups.telescopeSeconds = 0.0d;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

