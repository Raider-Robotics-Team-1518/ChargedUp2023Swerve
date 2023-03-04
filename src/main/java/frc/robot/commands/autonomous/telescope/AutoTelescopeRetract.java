package frc.robot.commands.autonomous.telescope;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoTelescopeRetract extends CommandBase {
    public AutoTelescopeRetract() {
        addRequirements(RobotContainer.swerveDrive, RobotContainer.armSubsystem, RobotContainer.clawSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.armSubsystem.telescopeRetract();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.armSubsystem.stopTelescope();
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.armSubsystem.telescopeRetracted();
    }
}
