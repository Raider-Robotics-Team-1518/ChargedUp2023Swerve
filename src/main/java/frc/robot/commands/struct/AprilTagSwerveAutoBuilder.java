package frc.robot.commands.struct;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class AprilTagSwerveAutoBuilder extends SwerveAutoBuilder{

    public Pose2d overrideStartPos;

    public AprilTagSwerveAutoBuilder(
        Supplier<Pose2d> poseSupplier,
        Consumer<Pose2d> resetPose,
        SwerveDriveKinematics kinematics,
        PIDConstants translationConstants,
        PIDConstants rotationConstants,
        Consumer<SwerveModuleState[]> outputModuleStates,
        Map<String, Command> eventMap,
        Subsystem... driveRequirements) {
    super(
        poseSupplier,
        resetPose,
        kinematics,
        translationConstants,
        rotationConstants,
        outputModuleStates,
        eventMap,
        false,
        driveRequirements);
    }


    @Override
    public CommandBase followPath(PathPlannerTrajectory trajectory) {
        return null;
    }


    @Override
    public CommandBase fullAuto(List<PathPlannerTrajectory> pathGroup) {
        List<CommandBase> commands = new ArrayList<>();

        // our code

        moveToPathStart(pathGroup, commands);

        // The rest is standard PathPlanner execution code
        commands.add(resetPose(pathGroup.get(0)));
    
        for (PathPlannerTrajectory traj : pathGroup) {
          commands.add(stopEventGroup(traj.getStartStopEvent()));
          commands.add(followPathWithEvents(traj));
        }
    
        commands.add(stopEventGroup(pathGroup.get(pathGroup.size() - 1).getEndStopEvent()));
    
        return Commands.sequence(commands.toArray(CommandBase[]::new));
    }

    public CommandBase onlyAprilTagMoveToStart(String pathName) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName, getPathConstraints());
        Pose2d currentPoseFieldRelative = getBotPosTeamRelative();
        Pose2d fixedCurrentPos = fieldRelativeToStandard(currentPoseFieldRelative);
        Translation2d translationDifference = getTranslationDifference(new Translation2d(trajectory.getInitialPose().getX(), trajectory.getInitialPose().getY()), new Translation2d(fixedCurrentPos.getX(), fixedCurrentPos.getY()));
        PathPlannerTrajectory wantedTrajectory = PathPlanner.generatePath(getPathConstraints(), 
            minimalisticPoint(zeroTranslation()),
            minimalisticPoint(translationDifference));
        return followPathWithEvents(wantedTrajectory);
    }

    public void resetOdometryAprilTag() {
        RobotContainer.swerveDrive.driveOdometry.resetPosition(Rotation2d.fromDegrees(0.0d), RobotContainer.swerveDrive.getSwerveModulePositions(), getBotPosTeamRelative());
    }



    private void moveToPathStart(List<PathPlannerTrajectory> pathGroup, List<CommandBase> commands) {
        Pose2d currentPoseFieldRelative = getBotPosTeamRelative();
        // offset by center of field values to make it so that (0,0) is the bottom left corner
        // this is needed because PathPlanner is not in terms of center of field
        Pose2d fixedCurrentPos = fieldRelativeToStandard(currentPoseFieldRelative);
        // perform the movement to starting position using AprilTags and Limelight
        // This is used so that the bot can start at any position as long as an AprilTag is recognized
        // by using those AprilTags we can get the bot's actual position and convert that
        // to be able to be used by PathPlanner

        if(currentPoseFieldRelative != null && fixedCurrentPos != null) {
            Pose2d firstPathPose = pathGroup.get(0).getInitialPose();
            Translation2d translationDifference = getTranslationDifference(new Translation2d(firstPathPose.getX(), firstPathPose.getY()), new Translation2d(fixedCurrentPos.getX(), fixedCurrentPos.getY()));
            PathPlannerTrajectory aprilTagTrajectory = PathPlanner.generatePath(
                getPathConstraints(),
                minimalisticPoint(zeroTranslation()),
                minimalisticPoint(translationDifference));
            simpleTrajectoryFollow(aprilTagTrajectory, commands);
        }
    }

    private PathPoint minimalisticPoint(Translation2d translation2d) {
        return new PathPoint(translation2d, Rotation2d.fromDegrees(0.0d));
    }

    private PathConstraints getPathConstraints() {
       return new PathConstraints(Constants.PATH_MAXIMUM_VELOCITY, Constants.PATH_MAXIMUM_ACCELERATION); 
    }

    private Translation2d zeroTranslation() {
        return new Translation2d(0.0d, 0.0d);
    }

    private Translation2d getTranslationDifference(Translation2d start, Translation2d stop) {
        Translation2d translation = start.minus(stop);
        return translation;
    }

    private void simpleTrajectoryFollow(PathPlannerTrajectory trajectory, List<CommandBase> commands) {
        // trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());
        // TODO: ^ !! NEEDED FOR COMPETITON !!

        // use this instead of resetPose command, seems to work more accurately for what we are doing
        RobotContainer.swerveDrive.driveOdometry.resetPosition(trajectory.getInitialState().holonomicRotation, RobotContainer.swerveDrive.getSwerveModulePositions(), trajectory.getInitialState().poseMeters);
        // commands.add(resetPose(trajectory));
        commands.add(stopEventGroup(trajectory.getStartStopEvent()));
        commands.add(followPathWithEvents(trajectory));
    }

    private Pose2d getBotPosTeamRelative() {
        Pose2d botPos = DriverStation.getAlliance() == Alliance.Blue ? LimelightHelpers.getBotPose2d_wpiRed("limelight") 
            : LimelightHelpers.getBotPose2d_wpiRed("limelight");
        return botPos;
    }

    private Pose2d fieldRelativeToStandard(Pose2d pose2d) {
        Pose2d newPos = pose2d.transformBy(new Transform2d(new Translation2d(-Constants.FIELD_CENTER_X, -Constants.FIELD_CENTER_Y), pose2d.getRotation().unaryMinus()));
        return newPos;
    }
}
