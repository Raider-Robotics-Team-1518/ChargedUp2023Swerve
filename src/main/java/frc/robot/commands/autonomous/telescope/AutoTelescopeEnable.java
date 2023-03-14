package frc.robot.commands.autonomous.telescope;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoTelescopeEnable extends CommandBase {
    private final double speed;
    public AutoTelescopeEnable(double speed) {
        this.speed = speed;
        addRequirements(RobotContainer.swerveDrive, RobotContainer.armSubsystem, RobotContainer.clawSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.armSubsystem.setTelescopeSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
