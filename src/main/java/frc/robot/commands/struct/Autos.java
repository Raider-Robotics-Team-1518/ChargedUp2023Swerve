package frc.robot.commands.struct;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.base.SwerveDrive;

public class Autos {
    public static Command getFullAuto(String pathName, double maxVel, double maxAccel) {
        List<PathPlannerTrajectory> pathGroup;
        pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(maxVel, maxAccel));

        SwerveDrive driveSystem = RobotContainer.swerveDrive;
        
        //AprilTagSwerveAutoBuilder autoBuilder = new AprilTagSwerveAutoBuilder(
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            driveSystem::getCurPose2d,
            driveSystem::setCurPose2d,
            driveSystem.driveKinematics,
            new PIDConstants(Constants.SWERVE_DRIVE_P_VALUE, Constants.SWERVE_DRIVE_I_VALUE, Constants.SWERVE_DRIVE_D_VALUE), 
            new PIDConstants(Constants.SWERVE_ROT_P_VALUE, Constants.SWERVE_ROT_I_VALUE, Constants.SWERVE_ROT_D_VALUE),
            driveSystem::setModuleStates,
            Constants.autonomousEventMap,
            driveSystem);

        autoBuilder.resetPose(pathGroup.get(0));
        return autoBuilder.fullAuto(pathGroup);
    }

    public static Command autoDriveStraight() {
        return getFullAuto("DriveStraight", Constants.PATH_MAXIMUM_VELOCITY, Constants.PATH_MAXIMUM_ACCELERATION);
    }

    public static Command autoHorseshoeTest() {
        return getFullAuto("DriveHorseshoeTest", Constants.PATH_MAXIMUM_VELOCITY, Constants.PATH_MAXIMUM_ACCELERATION);
    }

    public static Command autoSpinSlide() {
        return getFullAuto("180SpinSlide", Constants.PATH_MAXIMUM_VELOCITY, Constants.PATH_MAXIMUM_ACCELERATION);
    }
}
