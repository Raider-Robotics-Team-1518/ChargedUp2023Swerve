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
    private static final SwerveDrive driveSystem = RobotContainer.swerveDrive;
    private static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            driveSystem::getCurPose2d,
            driveSystem::setCurPose2d,
            driveSystem.driveKinematics,
            new PIDConstants(Constants.SWERVE_DRIVE_P_VALUE, Constants.SWERVE_DRIVE_I_VALUE, Constants.SWERVE_DRIVE_D_VALUE), 
            new PIDConstants(Constants.SWERVE_ROT_P_VALUE, Constants.SWERVE_ROT_I_VALUE, Constants.SWERVE_ROT_D_VALUE),
            driveSystem::setModulesVelocityToDutyCycle,
            Constants.autonomousEventMap,
            driveSystem);

    private static final AprilTagSwerveAutoBuilder aprilTagAutoBuilder = new AprilTagSwerveAutoBuilder(
            driveSystem::getCurPose2d,
            driveSystem::setCurPose2d,
            driveSystem.driveKinematics,
            new PIDConstants(Constants.SWERVE_DRIVE_P_VALUE, Constants.SWERVE_DRIVE_I_VALUE, Constants.SWERVE_DRIVE_D_VALUE), 
            new PIDConstants(Constants.SWERVE_ROT_P_VALUE, Constants.SWERVE_ROT_I_VALUE, Constants.SWERVE_ROT_D_VALUE),
            driveSystem::setModulesVelocityToDutyCycle,
            Constants.autonomousEventMap,
            driveSystem);

    public static Command getFullAuto(String pathName) {
        List<PathPlannerTrajectory> pathGroup;
        pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(Constants.PATH_MAXIMUM_VELOCITY, Constants.PATH_MAXIMUM_ACCELERATION));
        RobotContainer.swerveDrive.driveOdometry.resetPosition(pathGroup.get(0).getInitialState().holonomicRotation, RobotContainer.swerveDrive.getSwerveModulePositions(), pathGroup.get(0).getInitialState().poseMeters);
        return autoBuilder.fullAuto(pathGroup);
    }

    public static Command getFullAutoAprilTagStart(String pathName) {
        List<PathPlannerTrajectory> pathGroup;
        pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(Constants.PATH_MAXIMUM_VELOCITY, Constants.PATH_MAXIMUM_ACCELERATION));
        RobotContainer.swerveDrive.driveOdometry.resetPosition(pathGroup.get(0).getInitialState().holonomicRotation, RobotContainer.swerveDrive.getSwerveModulePositions(), pathGroup.get(0).getInitialState().poseMeters);
        return aprilTagAutoBuilder.fullAuto(pathGroup);
    }

    // For 1518: The heading will be reversed in PathPlanner (white/gold dot = rear of robot) because translations are offsetted improperly 
    // I don't really know why, but it does not really matter?


    // Basic tests
    public static Command autoDriveStraight() {
        return getFullAuto("DriveStraight");
    }

    public static Command autoHorseshoeTest() {
        return getFullAuto("DriveHorseshoeTest");
    }

    public static Command autoSpinSlide() {
        return getFullAuto("180SpinSlide");
    }

    // Initial get out of the way

    /*
     * (X, Y) End Positions for Out of way Paths
     * OOW1: (4.90, 0.78)
     * OOW2: (5.45, 2.85)
     * OOW3/OOWAdvanced: (5.74, 3.60)
     */
    
    public static Command outOfWayOne() {
        return aprilTagAutoBuilder.onlyAprilTagMoveToStart("OOW1");
    }

    public static Command outofWayTwo() {
        return aprilTagAutoBuilder.onlyAprilTagMoveToStart("OOW2");
    }

    // Normal AprilTag following path as we need a trajectory to avoid
    // the possibility of bumping an alliance member by going over
    // the charging station, so we must AVOID THE CHARGE STATION if we are starting up top
    public static Command outOfWayThree() {
        return getFullAutoAprilTagStart("OOWAdvanced");
    }

}
