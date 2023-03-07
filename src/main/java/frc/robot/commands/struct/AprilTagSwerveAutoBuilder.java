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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
        
        Pose2d actualCurrentPosition = LimelightHelpers.getBotPose2d("limelight");
        if(actualCurrentPosition != null) {
            Pose2d initialPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
            Pose2d endPose = pathGroup.get(0).getInitialPose();
            Translation2d differentialOffset = endPose.minus(initialPose).getTranslation();
            PathPlannerTrajectory aprilTagTrajectory = PathPlanner.generatePath(
                new PathConstraints(Constants.PATH_MAXIMUM_VELOCITY, Constants.MAXIMUM_ACCELERATION),
                new PathPoint(initialPose.getTranslation(), Rotation2d.fromDegrees(0)),
                new PathPoint(differentialOffset, Rotation2d.fromDegrees(90)
            ));
            commands.add(resetPose(aprilTagTrajectory));
            commands.add(followPathWithEvents(aprilTagTrajectory));
        }
        commands.add(resetPose(pathGroup.get(0)));
    
        for (PathPlannerTrajectory traj : pathGroup) {
          commands.add(stopEventGroup(traj.getStartStopEvent()));
          commands.add(followPathWithEvents(traj));
        }
    
        commands.add(stopEventGroup(pathGroup.get(pathGroup.size() - 1).getEndStopEvent()));
    
        return Commands.sequence(commands.toArray(CommandBase[]::new));
    }
}
