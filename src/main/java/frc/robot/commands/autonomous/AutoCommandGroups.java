package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.PlaceMode;
import frc.robot.commands.autonomous.shoulder.AutoRotateShoulderCarrying;
import frc.robot.commands.autonomous.shoulder.AutoRotateShoulderGrabbing;
import frc.robot.commands.autonomous.shoulder.AutoRotateShoulderIdle;
import frc.robot.commands.autonomous.shoulder.AutoRotateShoulderPlacing;
import frc.robot.commands.autonomous.shoulder.AutoRotateShoulderToAngle;
import frc.robot.commands.autonomous.telescope.AutoTelescopeDisable;
import frc.robot.commands.autonomous.telescope.AutoTelescopeEnable;
import frc.robot.commands.autonomous.telescope.ResetTelescopeSeconds;
import frc.robot.commands.autonomous.telescope.SetTelescopeSeconds;
import frc.robot.commands.struct.Autos;

public class AutoCommandGroups {

    public static double telescopeSeconds = 0.0d;

    public static SequentialCommandGroup rotateArmGrabbing() {
        double telescopeSecondsGrabbing = 0.25d;
        telescopeSeconds+=telescopeSecondsGrabbing;
        return new SequentialCommandGroup(
            new AutoRotateShoulderGrabbing(),
            new WaitCommand(0.5d),

            new AutoTelescopeEnable(1.0d),
            new WaitCommand(telescopeSecondsGrabbing),
            new AutoTelescopeDisable()
        );
    }

    public static SequentialCommandGroup rotateArmCarrying() {
        double telescopeSecondsCarrying = 0.25d;
        return new SequentialCommandGroup(
            retractTelescope(),
            new AutoRotateShoulderCarrying(),
            new WaitCommand(0.5d),
            new AutoTelescopeEnable(1.0d),
            new WaitCommand(telescopeSecondsCarrying),
            new AutoTelescopeDisable(),
            new SetTelescopeSeconds(telescopeSecondsCarrying)
        );
    }

    public static SequentialCommandGroup retractTelescope() {
        return new SequentialCommandGroup(
            new AutoTelescopeEnable(-1d),
            new WaitCommand(telescopeSeconds),
            new AutoTelescopeDisable(),
            new ResetTelescopeSeconds()
        );
    }

    public static SequentialCommandGroup rotateArmPlacing(PlaceMode placeMode) {
        return new SequentialCommandGroup(
            new AutoRotateShoulderPlacing(placeMode),
            new WaitCommand(0.5d),
            moveTelescopePlacing(placeMode)
        );
    }

    public static SequentialCommandGroup rotateArmIdle() {
        return new SequentialCommandGroup(
            retractTelescope(),
            new AutoRotateShoulderIdle()
        );
    }

    public static SequentialCommandGroup moveTelescopePlacing(PlaceMode placeMode) {
        telescopeSeconds+=placeMode.getWaitTime();
        return new SequentialCommandGroup(
            new AutoTelescopeEnable(1d),
            new WaitCommand(placeMode.getWaitTime()),
            new AutoTelescopeDisable()
        );
    }

    public static Command driveBackAuto() {
        return Autos.getFullAuto("DriveBack");
    }

    public static SequentialCommandGroup doAutonomousScoreOne(String pathName) {
        String after = RobotContainer.csChooser.getSelected();
        if(after == null) after = "Mid";
        String override = RobotContainer.csChooserOverride.getSelected();
        if(override == null || override.equalsIgnoreCase("none")) {
            override = after;
        }
        return new SequentialCommandGroup(
            Autos.getFullAutoAprilTagStart(pathName+after),
            Autos.getFullAutoAprilTagStart("ChargeStation"+override) // end auto go to charge station
            );
    }


    // debugging

    public static SequentialCommandGroup moveTelescopeSecondsTest() {
        double seconds = SmartDashboard.getNumber("TelescopeSeconds", 0.0d);
        telescopeSeconds+=seconds;
        return new SequentialCommandGroup(
            new AutoTelescopeEnable(1d),
            new WaitCommand(seconds),
            new AutoTelescopeDisable()
        );
    }

    public static CommandBase moveShoulderTest() {
        double angle = SmartDashboard.getNumber("WantedAnglee", Constants.ARM_SHOULDER_LOWERSWITCH_DEG);
        return new AutoRotateShoulderToAngle(angle);
    }
}
