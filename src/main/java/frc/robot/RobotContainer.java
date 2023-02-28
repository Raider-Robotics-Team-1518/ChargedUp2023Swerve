// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.DriveRobotCentric;
import frc.robot.commands.drive.DriveStopAllModules;
import frc.robot.commands.drive.util.DriveAdjustModulesManually;
import frc.robot.commands.drive.util.DriveAllModulesPositionOnly;
import frc.robot.commands.drive.util.DriveOneModule;
import frc.robot.commands.drive.util.DriveResetAllModulePositionsToZero;
import frc.robot.commands.drive.util.DriveTurnToAngleInRad;
import frc.robot.commands.operational.claw.ClawMove;
import frc.robot.commands.operational.claw.ClawStop;
import frc.robot.commands.operational.setup.general.SetArmBrake;
import frc.robot.commands.operational.setup.general.SetArmCoast;
import frc.robot.commands.operational.setup.general.SetupToggle;
import frc.robot.commands.operational.setup.shoulder.ShoulderExportData;
import frc.robot.commands.operational.setup.shoulder.ShoulderSetIdle;
import frc.robot.commands.operational.setup.shoulder.ShoulderSetMaxMin;
import frc.robot.commands.operational.setup.shoulder.ShoulderSetZero;
import frc.robot.commands.operational.setup.shoulder.ShoulderSetup;
import frc.robot.commands.operational.setup.telescope.TelescopeSetMax;
import frc.robot.commands.operational.setup.telescope.TelescopeSetMin;
import frc.robot.commands.operational.setup.telescope.TelescopeSetup;
import frc.robot.commands.operational.setup.wrist.WristExportData;
import frc.robot.commands.operational.setup.wrist.WristSetIdle;
import frc.robot.commands.operational.setup.wrist.WristSetMax;
import frc.robot.commands.operational.setup.wrist.WristSetMin;
import frc.robot.commands.operational.setup.wrist.WristSetup;
import frc.robot.commands.operational.telescope.TelescopeMove;
import frc.robot.commands.operational.telescope.TelescopeStop;
import frc.robot.subsystems.base.SwerveDrive;
import frc.robot.subsystems.moving.ArmSubsystem;
import frc.robot.subsystems.moving.ClawSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's gamepads are defined here...
  
  static final XboxController driver = new XboxController(0);
  static final XboxController coDriver = new XboxController(1);

  ////////////////////
  // DRIVER BUTTONS //
  ////////////////////

  static final Trigger driverA = new JoystickButton(driver, 1);
  static final Trigger driverB = new JoystickButton(driver, 2);
  static final Trigger driverX = new JoystickButton(driver, 3);
  static final Trigger driverY = new JoystickButton(driver, 4);
  static final Trigger driverLB = new JoystickButton(driver, 5);
  static final Trigger driverRB = new JoystickButton(driver, 6);
  static final Trigger driverBack = new JoystickButton(driver, 7);
  static final Trigger driverStart = new JoystickButton(driver, 8);
  static final Trigger driverLS = new JoystickButton(driver, 9);
  static final Trigger driverRS = new JoystickButton(driver, 10);
  static final Trigger driverDUp = new POVButton(driver, 0);
  static final Trigger driverDDown = new POVButton(driver, 180);
  static final Trigger driverDLeft = new POVButton(driver, 270);
  static final Trigger driverDRight = new POVButton(driver, 90);

  ///////////////////////
  // CO-DRIVER BUTTONS //
  ///////////////////////

  static final Trigger coDriverA = new JoystickButton(coDriver, 1);
  static final Trigger coDriverB = new JoystickButton(coDriver, 2);
  static final Trigger coDriverX = new JoystickButton(coDriver, 3);
  static final Trigger coDriverY = new JoystickButton(coDriver, 4);
  static final Trigger coDriverLB = new JoystickButton(coDriver, 5);
  static final Trigger coDriverRB = new JoystickButton(coDriver, 6);
  static final Trigger coDriverBack = new JoystickButton(coDriver, 7);
  static final Trigger coDriverStart = new JoystickButton(coDriver, 8);
  static final Trigger coDriverLS = new JoystickButton(coDriver, 9);
  static final Trigger coDriverRS = new JoystickButton(coDriver, 10);
  static final Trigger coDriverDUp = new POVButton(coDriver, 0);
  static final Trigger coDriverDDown = new POVButton(coDriver, 180);
  static final Trigger coDriverDLeft = new POVButton(coDriver, 270);
  static final Trigger coDriverDRight = new POVButton(coDriver, 90);


  //The robot's subsystems are instantiated here
  public static SwerveDrive swerveDrive;
  public static ArmSubsystem armSubsystem;
  public static ClawSubsystem clawSubsystem;

  /* Command Choosers */
  public static SendableChooser<Command> autoChooser = new SendableChooser<Command>(); // Autonomous
  public static SendableChooser<Command> setupSwerveChooser = new SendableChooser<Command>(); // Swerve Setup
  public static SendableChooser<Command> setupShoulderChooser = new SendableChooser<Command>(); // Shoulder Setup
  public static SendableChooser<Command> setupWristChooser = new SendableChooser<Command>(); // Wrist Setup
  public static SendableChooser<Command> setupTelescopeChooser = new SendableChooser<Command>(); // Telescope Setup


  public RobotContainer() {
    armSubsystem = new ArmSubsystem();
    clawSubsystem = new ClawSubsystem();
    swerveDrive = new SwerveDrive();
    swerveDrive.setDefaultCommand(new DriveRobotCentric(false));

    configureSwerveSetup();
    configureSetupModes();
    configureAutoModes();
    configureButtonBindings();
    
    CameraServer.startAutomaticCapture();
  }


  private void configureButtonBindings() {
    /* ==================== DRIVER BUTTONS ==================== 

    driverLB.onTrue(new DriveResetGyroToZero());
    driverA.whileTrue(new ShoulderMoveOffset(0.5));
    driverB.whileTrue(new ShoulderMoveOffset(-0.5));

    driverStart.toggleOnTrue(new DriveFieldRelative(false));
    driverBack.toggleOnTrue(new DriveFieldRelativeAdvanced(false));
    //driverBack.or(driverStart).toggleOnTrue(new DriveFieldRelative(false));

    /* =================== CODRIVER BUTTONS =================== 

    coDriverA.whileTrue(new ShoulderMoveOffset(0.75));
    coDriverB.whileTrue(new ShoulderMoveOffset(-0.75));

    coDriverX.whileTrue(new TelescopeMove(0.25d)).onFalse(new TelescopeStop());
    coDriverY.whileTrue(new TelescopeMove(-0.25d)).onFalse(new TelescopeStop());
    
    coDriverDUp.whileTrue(new WristMove(0.25d)).onFalse(new WristStop());
    coDriverDDown.whileTrue(new WristMove(-0.25d)).onFalse(new WristStop());

    coDriverBack.toggleOnTrue(new WristToggleLock());

    coDriverRB.whileTrue(new ClawMove(0.25d)).onFalse(new ClawStop());
    coDriverLB.whileTrue(new ClawMove(-0.25d)).onFalse(new ClawStop());*/
    driverRB.whileTrue(new ClawMove(0.25d)).onFalse(new ClawStop());
    driverLB.whileTrue(new ClawMove(-0.25d)).onFalse(new ClawStop());

    driverX.whileTrue(Commands.runOnce(() -> armSubsystem.getWristMotor().set(0.5d))).onFalse(Commands.runOnce(() -> armSubsystem.getWristMotor().set(0.0d)));
    driverY.whileTrue(Commands.runOnce(() -> armSubsystem.getWristMotor().set(-0.5d))).onFalse(Commands.runOnce(() -> armSubsystem.getWristMotor().set(0.0d)));

    //driverA.toggleOnTrue(Commands.runOnce(() -> Constants.ARM_TELESCOPE_GRAVITY_FACTOR=Constants.ARM_TELESCOPE_GRAVITY_FACTOR+0.001d));
    //driverB.toggleOnTrue(Commands.runOnce(() -> Constants.ARM_TELESCOPE_GRAVITY_FACTOR=Constants.ARM_TELESCOPE_GRAVITY_FACTOR-0.001d));
  }



  /**
   * Define all autonomous modes here to have them 
   * appear in the autonomous select drop down menu.
   * They will appear in the order entered
   */
  private void configureAutoModes() {
    
    autoChooser.setDefaultOption("Wait 1 sec(do nothing)", new WaitCommand(1));


    SmartDashboard.putData(RobotContainer.autoChooser);
  }

  private void configureSwerveSetup() {
    Preferences.remove(Constants.SHOULDER_IDLE_ANGLE);
    Preferences.remove(Constants.SHOULDER_LEVEL_POS);
    Preferences.remove(Constants.SHOULDER_MAX_POS);
    Preferences.remove(Constants.SHOULDER_MIN_POS);
    Preferences.remove(Constants.WRIST_IDLE_ANGLE);
    Preferences.remove(Constants.WRIST_MAX_POS);
    Preferences.remove(Constants.WRIST_MIN_POS);
    Preferences.remove(Constants.TELESCOPE_MAX_POS);
    Preferences.remove(Constants.TELESCOPE_MIN_POS);



    setupSwerveChooser.addOption("D Physical", new DriveAdjustModulesManually());
    setupSwerveChooser.addOption("D To Zero", new DriveResetAllModulePositionsToZero());
    setupSwerveChooser.addOption("D Module 0", new DriveOneModule(0));
    setupSwerveChooser.addOption("D Module 1", new DriveOneModule(1));
    setupSwerveChooser.addOption("D Module 2", new DriveOneModule(2));
    setupSwerveChooser.addOption("D Module 3", new DriveOneModule(3));
    setupSwerveChooser.addOption("D Pos Only", new DriveAllModulesPositionOnly());
    setupSwerveChooser.addOption("D Stop", new DriveStopAllModules());
    setupSwerveChooser.addOption("Turn to Angle", new DriveTurnToAngleInRad(Constants.PI_OVER_TWO));
    SmartDashboard.putData(setupSwerveChooser);
  }

  private void configureSetupModes() {
    /* General Setup Commands */
    SmartDashboard.putData(new SetupToggle());
    SmartDashboard.putData(new ShoulderSetup());
    SmartDashboard.putData(new TelescopeSetup());
    SmartDashboard.putData(new WristSetup());
    SmartDashboard.putData(new SetArmBrake());
    SmartDashboard.putData(new SetArmCoast());

    /* Shoulder */
    setupShoulderChooser.addOption("Find Max/Mins", new ShoulderSetMaxMin());
    setupShoulderChooser.addOption("Export Motion Data", new ShoulderExportData());
    setupShoulderChooser.addOption("Set Idle Pos", new ShoulderSetIdle());
    setupShoulderChooser.addOption("Set Level Pos", new ShoulderSetZero());
    setupShoulderChooser.setDefaultOption("None", new WaitCommand(1));
    SmartDashboard.putData(setupShoulderChooser);

    /* Wrist */
    // setupWristChooser.addOption("Find Max/Mins", new WristSetMaxMin()); // Read comment at top of class
    setupWristChooser.addOption("Set Max (90 deg)", new WristSetMax());
    setupWristChooser.addOption("Set Idle Pos", new WristSetIdle());
    setupWristChooser.addOption("Set Min (Level)", new WristSetMin());
    setupWristChooser.addOption("Export Motion Data", new WristExportData());
    setupWristChooser.setDefaultOption("None", new WaitCommand(1));
    SmartDashboard.putData(setupWristChooser);


    /* Telescope */
    setupTelescopeChooser.addOption("Set Min (In)", new TelescopeSetMin());
    setupTelescopeChooser.addOption("Set Max (Out)", new TelescopeSetMax());
    setupTelescopeChooser.setDefaultOption("None", new WaitCommand(1));
    SmartDashboard.putData(setupTelescopeChooser);

  }

  
  /**
   * A method to return the value of a driver joystick axis,
   * which runs from -1.0 to 1.0, with a .1 dead zone(a 0 
   * value returned if the joystick value is between -.1 and 
   * .1)
   * @param axis
   * @return value of the joystick, from -1.0 to 1.0 where 0.0 is centered
   */
  public double getDriverAxis(Axis axis) {
    return (driver.getRawAxis(axis.value) < -.1 || driver.getRawAxis(axis.value) > .1)
        ? driver.getRawAxis(axis.value)
        : 0.0;
  }

  /**
   * Accessor method to set driver rumble function
   * 
   * @param leftRumble
   * @param rightRumble
   */
  public static void setDriverRumble(double leftRumble, double rightRumble) {
    driver.setRumble(RumbleType.kLeftRumble, leftRumble);
    driver.setRumble(RumbleType.kRightRumble, rightRumble);
  }

  /**
   * Returns the int position of the DPad/POVhat based
   * on the following table:
   *    input    |return
   * not pressed |  -1
   *     up      |   0
   *   up right  |  45
   *    right    |  90
   *  down right | 135
   *    down     | 180
   *  down left  | 225
   *    left     | 270
   *   up left   | 315
   * @return
   */
  public int getDriverDPad() {
    return (driver.getPOV());
  }

  /**
   * A method to return the value of a codriver joystick axis,
   * which runs from -1.0 to 1.0, with a .1 dead zone(a 0 
   * value returned if the joystick value is between -.1 and 
   * .1) 
   * @param axis
   * @return
   */
  public double getCoDriverAxis(Axis axis) {
    return (coDriver.getRawAxis(axis.value) < -.1 || coDriver.getRawAxis(axis.value) > .1)
        ? coDriver.getRawAxis(axis.value)
        : 0;
  }

  /**
   * Accessor method to set codriver rumble function
   * 
   * @param leftRumble
   * @param rightRumble
   */
  public static void setCoDriverRumble(double leftRumble, double rightRumble) {
    coDriver.setRumble(RumbleType.kLeftRumble, leftRumble);
    coDriver.setRumble(RumbleType.kRightRumble, rightRumble);
  }

  /**
   * accessor to get the true/false of the buttonNum 
   * on the coDriver control
   * @param buttonNum
   * @return the value of the button
   */
  public boolean getCoDriverButton(int buttonNum) {
    return coDriver.getRawButton(buttonNum);
  }

}
