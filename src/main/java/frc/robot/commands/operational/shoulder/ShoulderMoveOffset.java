package frc.robot.commands.operational.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/*
 * Move the shoulder by a rotational encoder offset
 * Uses PID to automatically calculate motor speeds and correct the position
 */
public class ShoulderMoveOffset extends CommandBase {
    public boolean done;
    public double opp = 0;
    public static ShoulderMoveOffset INSTANCE;
    public ShoulderMoveOffset(double offset) {
        INSTANCE = this;
        this.done = false;
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
        done = false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
  
    @Override
    public boolean isFinished() {
        return done;
    }
  }

