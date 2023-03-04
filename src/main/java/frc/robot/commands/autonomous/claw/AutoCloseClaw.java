package frc.robot.commands.autonomous.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoCloseClaw extends CommandBase {
    public AutoCloseClaw() {
        addRequirements(RobotContainer.swerveDrive, RobotContainer.armSubsystem, RobotContainer.clawSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.clawSubsystem.enableClawMotor(-Constants.autoClawSpeed);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.clawSubsystem.innerSwitchReached();
    }
}
