package frc.robot.commands.autonomous;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/*
 * Customly structured class for ease of implentation of commands that REQUIRE PathPlanner
 * in any way shape or form.
 */
public class PathCommand extends CommandBase {
    List<PathPlannerTrajectory> pathGroup;
    public PathCommand(String pathName, double maxVelocity, double maxAcceleration) {
            pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(maxVelocity, maxAcceleration));
        addRequirements(RobotContainer.swerveDrive, RobotContainer.armSubsystem, RobotContainer.clawSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
    }
  
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
  
    @Override
    public boolean isFinished() {
        return false;
    }
}
