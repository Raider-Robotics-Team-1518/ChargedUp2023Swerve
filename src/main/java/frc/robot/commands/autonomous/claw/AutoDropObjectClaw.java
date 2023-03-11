package frc.robot.commands.autonomous.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoDropObjectClaw extends CommandBase {
    public AutoDropObjectClaw() {
        addRequirements(RobotContainer.swerveDrive, RobotContainer.armSubsystem, RobotContainer.clawSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.clawSubsystem.setClawSpeed(-Constants.clawDropSpeed);
        Commands.waitSeconds(1);
        RobotContainer.clawSubsystem.setClawSpeed(0.0d);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
