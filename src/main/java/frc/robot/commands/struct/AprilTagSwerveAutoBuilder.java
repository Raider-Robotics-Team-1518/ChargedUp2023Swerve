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

public class AprilTagSwerveAutoBuilder extends SwerveAutoBuilder{

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
        
        Pose2d currentPoseFieldRelative = 
            DriverStation.getAlliance() == Alliance.Blue ? LimelightHelpers.getBotPose2d_wpiRed("limelight") 
            : LimelightHelpers.getBotPose2d_wpiRed("limelight");

        // offset by center of field values to make it so that (0,0) is the bottom left corner
        // this is needed because PathPlanner is not in terms of center of field
        currentPoseFieldRelative.transformBy(new Transform2d(new Translation2d(0, 0), currentPoseFieldRelative.getRotation().unaryMinus()));
        Pose2d fixedCurrentPos = currentPoseFieldRelative.transformBy(new Transform2d(new Translation2d(-Constants.FIELD_CENTER_X, -Constants.FIELD_CENTER_Y), Rotation2d.fromDegrees(0.0d)));
        // perform the movement to starting position using AprilTags and Limelight
        // This is used so that the bot can start at any position as long as an AprilTag is recognized
        // by using those AprilTags we can get the bot's actual position and convert that
        // to be able to be used by PathPlanner

        if(currentPoseFieldRelative != null && fixedCurrentPos != null) {
            Pose2d firstPathPose = pathGroup.get(0).getInitialPose();
            Translation2d firstPoint = new Translation2d(firstPathPose.getX(), firstPathPose.getY());
            Translation2d actualPos = new Translation2d(fixedCurrentPos.getX(), fixedCurrentPos.getY());
            Translation2d differenceTranslation2d = firstPoint.minus(actualPos);
            PathPlannerTrajectory aprilTagTrajectory = PathPlanner.generatePath(
                new PathConstraints(Constants.PATH_MAXIMUM_VELOCITY, Constants.PATH_MAXIMUM_ACCELERATION),
                new PathPoint(new Translation2d(0.0d, 0.0d), Rotation2d.fromDegrees(0.0d)),
                new PathPoint(differenceTranslation2d, Rotation2d.fromDegrees(0.0d)));
            commands.add(resetPose(aprilTagTrajectory));
            commands.add(followPathWithEvents(aprilTagTrajectory));
        }

        // The rest is standard PathPlanner execution code
        commands.add(resetPose(pathGroup.get(0)));
    
        for (PathPlannerTrajectory traj : pathGroup) {
          commands.add(stopEventGroup(traj.getStartStopEvent()));
          commands.add(followPathWithEvents(traj));
        }
    
        commands.add(stopEventGroup(pathGroup.get(pathGroup.size() - 1).getEndStopEvent()));
    
        return Commands.sequence(commands.toArray(CommandBase[]::new));
    }
}
