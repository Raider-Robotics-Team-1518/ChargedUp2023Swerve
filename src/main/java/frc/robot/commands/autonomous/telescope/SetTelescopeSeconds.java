package frc.robot.commands.autonomous.telescope;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.autonomous.AutoCommandGroups;

public class SetTelescopeSeconds extends CommandBase {
    private final double secs;
    public SetTelescopeSeconds(double secs) {
        this.secs = secs;
        addRequirements(RobotContainer.swerveDrive, RobotContainer.armSubsystem, RobotContainer.clawSubsystem);
    }

    @Override
    public void execute() {
        AutoCommandGroups.telescopeSeconds = secs;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
