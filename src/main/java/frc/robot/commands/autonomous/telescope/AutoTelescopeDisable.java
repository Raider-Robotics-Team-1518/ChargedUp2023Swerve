package frc.robot.commands.autonomous.telescope;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoTelescopeDisable extends CommandBase {
    public AutoTelescopeDisable() {
        addRequirements(RobotContainer.swerveDrive, RobotContainer.armSubsystem, RobotContainer.clawSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.armSubsystem.setTelescopeSpeed(0.0d);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
