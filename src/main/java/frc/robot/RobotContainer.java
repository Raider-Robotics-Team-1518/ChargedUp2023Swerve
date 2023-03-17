// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
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
import frc.robot.Constants.PlaceMode;
import frc.robot.commands.autonomous.AutoCommandGroups;
import frc.robot.commands.autonomous.claw.AutoDropObjectClaw;
import frc.robot.commands.autonomous.claw.AutoFeedClaw;
import frc.robot.commands.drive.DriveFieldRelative;
import frc.robot.commands.drive.DriveRobotCentric;
import frc.robot.commands.drive.DriveRobotCentricDPAD;
import frc.robot.commands.drive.DriveStopAllModules;
import frc.robot.commands.drive.util.DriveAdjustModulesManually;
import frc.robot.commands.drive.util.DriveAllModulesPositionOnly;
import frc.robot.commands.drive.util.DriveOneModule;
import frc.robot.commands.drive.util.DriveResetAllModulePositionsToZero;
import frc.robot.commands.drive.util.DriveTurnToAngleInRad;
import frc.robot.commands.drive.util.pid.DriveRotationExport;
import frc.robot.commands.drive.util.pid.DriveTranslationExport;
import frc.robot.commands.operational.claw.ClawMove;
import frc.robot.commands.operational.claw.ClawStop;
import frc.robot.commands.operational.setup.general.SetArmBrake;
import frc.robot.commands.operational.setup.general.SetArmCoast;
import frc.robot.commands.operational.setup.general.SetupToggle;
import frc.robot.commands.operational.setup.shoulder.ShoulderExportData;
import frc.robot.commands.operational.setup.shoulder.ShoulderMoveLevel;
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
import frc.robot.commands.operational.shoulder.ShoulderMoveOffset;
import frc.robot.commands.operational.telescope.TelescopeMove;
import frc.robot.commands.operational.telescope.TelescopeStop;
import frc.robot.commands.operational.wrist.WristMove;
import frc.robot.commands.operational.wrist.WristStop;
import frc.robot.commands.struct.Autos;
import frc.robot.subsystems.base.Lights;
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
  public static SendableChooser<String> csChooser = new SendableChooser<String>(); // Autonomous ChargeStation Part
  public static SendableChooser<String> csChooserOverride = new SendableChooser<String>(); // Autonomous ChargeStation Part Override
  public static SendableChooser<Command> setupShoulderChooser = new SendableChooser<Command>(); // Shoulder Setup
  public static SendableChooser<Command> setupWristChooser = new SendableChooser<Command>(); // Wrist Setup
  public static SendableChooser<Command> setupTelescopeChooser = new SendableChooser<Command>(); // Telescope Setup

  /* LED Lights */
  public static Lights m_blinkies = new Lights();

  public RobotContainer() {
    armSubsystem = new ArmSubsystem();
    clawSubsystem = new ClawSubsystem();
    swerveDrive = new SwerveDrive();
    swerveDrive.setDefaultCommand(new DriveRobotCentric(false));

    configureSwerveSetup();
    configureSetupModes();
    configureAutoModes();
    configureButtonBindings();
    configureAutonomousEventMap();
    
    SmartDashboard.putData(new DriveAdjustModulesManually());
    CameraServer.startAutomaticCapture();
  }
    /*
     * (X,Y) End Positions for Out of way Paths
     * OOW1: (4.90, 0.78)
     * OOW2: (5.45, 2.85)
     * OOW3/OOWAdvanced: (5.74, 3.60)
     */

  private void configureAutonomousEventMap() {
    Constants.autonomousEventMap.put("intake", AutoCommandGroups.intakeItem());
    Constants.autonomousEventMap.put("puke", AutoCommandGroups.pukeItem());
    Constants.autonomousEventMap.put("retractTelescope", AutoCommandGroups.retractTelescope());
    Constants.autonomousEventMap.put("rotateArmIdle", AutoCommandGroups.rotateArmIdle());
    Constants.autonomousEventMap.put("rotateArmGrabbing", AutoCommandGroups.rotateArmGrabbing());
    Constants.autonomousEventMap.put("rotateArmCarrying", AutoCommandGroups.rotateArmCarrying());
    Constants.autonomousEventMap.put("rotatePlaceHigh", AutoCommandGroups.rotateArmPlacing(PlaceMode.HIGH_NODE_CUBE));
    Constants.autonomousEventMap.put("placeConeHigh", AutoCommandGroups.placeObject(PlaceMode.HIGH_NODE_CONE));
    Constants.autonomousEventMap.put("placeConeMid", AutoCommandGroups.placeObject(PlaceMode.MID_NODE_CONE));
    Constants.autonomousEventMap.put("placeConeLow", AutoCommandGroups.placeObject(PlaceMode.LOW_NODE_CONE));
    Constants.autonomousEventMap.put("placeCubeHigh", AutoCommandGroups.placeObject(PlaceMode.HIGH_NODE_CUBE));
    Constants.autonomousEventMap.put("placeCubeMid", AutoCommandGroups.placeObject(PlaceMode.MID_NODE_CUBE));
    Constants.autonomousEventMap.put("placeCubeLow", AutoCommandGroups.placeCubeLow());
  }


  private void configureButtonBindings() {
    /* ==================== DRIVER BUTTONS ==================== */

    //driverLB.onTrue(new DriveResetGyroToZero());
    driverStart.toggleOnTrue(new DriveRobotCentric(false));
    driverBack.toggleOnTrue(new DriveFieldRelative(false));
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
    //coDriverStart.toggleOnTrue(new WristToggleLock());
    coDriverX.whileTrue(new ClawMove(Constants.clawFeedSpeed)).onFalse(new ClawStop());
    coDriverY.whileTrue(new ClawMove(-Constants.clawDropSpeed)).onFalse(new ClawStop());
    //coDriverY.whileTrue(new ClawMove(Constants.clawSpeed)).onFalse(new ClawStop());
    //coDriverDUp.whileTrue(new WristMove(Constants.wristSpeed)).onFalse(new WristStop());
    coDriverDDown.whileTrue(new WristMove(-Constants.wristSpeed)).onFalse(new WristStop());
    coDriverRB.whileTrue(new TelescopeMove(Constants.telescopeSpeed)).onFalse(new TelescopeStop());
    coDriverLB.whileTrue(new TelescopeMove(-Constants.telescopeSpeed)).onFalse(new TelescopeStop());
    coDriverA.whileTrue(new ShoulderMoveOffset(Constants.shoulderUpSpeed));
    coDriverB.whileTrue(new ShoulderMoveOffset(-Constants.shoulderDownSpeed));

    //coDriverDUp.whileTrue(Commands.runOnce(() -> armSubsystem.getWristMotor().set(Constants.wristSpeed))).onFalse(Commands.runOnce(() -> armSubsystem.getWristMotor().set(0.0d)));
    //coDriverDDown.whileTrue(Commands.runOnce(() -> armSubsystem.getWristMotor().set(-Constants.wristSpeed))).onFalse(Commands.runOnce(() -> armSubsystem.getWristMotor().set(0.0d)));

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
    autoChooser.addOption("1Cube", AutoCommandGroups.doAutonomousScoreOne("Auto1Cube"));
    autoChooser.addOption("DriveBack", AutoCommandGroups.driveBackAuto());
    autoChooser.addOption("Score1Low", AutoCommandGroups.placeCubeLow());
    autoChooser.addOption("1Cone", AutoCommandGroups.doAutonomousScoreOne("Auto1Cone"));

    csChooser.setDefaultOption("Middle", "Mid");
    csChooser.addOption("Top", "Upper");
    csChooser.addOption("Middle", "Mid");
    csChooser.addOption("Bottom", "Lower");

    csChooserOverride.setDefaultOption("None", "none");
    csChooserOverride.addOption("Top", "Upper");
    csChooserOverride.addOption("Middle", "Mid");
    csChooserOverride.addOption("Bottom", "Lower");

    SmartDashboard.putData(RobotContainer.autoChooser);
    SmartDashboard.putData(RobotContainer.csChooser);
    SmartDashboard.putData(RobotContainer.csChooserOverride);
  }

  private void configureSwerveSetup() {
    //SmartDashboard.putData(new DriveAdjustModulesManually());
    SmartDashboard.putData(new DriveResetAllModulePositionsToZero());
    //SmartDashboard.putData(new DriveOneModule(0));
    //SmartDashboard.putData(new DriveOneModule(1));
    //SmartDashboard.putData(new DriveOneModule(2));
    //SmartDashboard.putData(new DriveOneModule(3));
    //SmartDashboard.putData(new DriveAllModulesPositionOnly());
    //SmartDashboard.putData(new DriveStopAllModules());
    //SmartDashboard.putData(new DriveTurnToAngleInRad(Constants.PI_OVER_TWO));

    // 1518
    SmartDashboard.putData(new DriveTranslationExport());
    SmartDashboard.putData(new DriveRotationExport());
    SmartDashboard.putData("Drive Straight", Commands.sequence(Autos.autoDriveStraight()));
    //SmartDashboard.putData("Drive Horseshoe", Commands.sequence(Autos.autoHorseshoeTest()));
    //SmartDashboard.putData("Auto Spin Slide", Commands.sequence(Autos.autoSpinSlide()));
  }

  private void configureSetupModes() {
    /*
     * General Debugging/Measuring
     */

    SmartDashboard.putNumber("TelescopeSeconds", 0.0d);
    SmartDashboard.putData("TestTelescopeSeconds", AutoCommandGroups.moveTelescopeSecondsTest());

    SmartDashboard.putNumber("WantedAnglee", 45d);
    SmartDashboard.putData("TestAngleSetpoint", AutoCommandGroups.moveShoulderTest());

    /* General Setup Commands */
    SmartDashboard.putData(new SetupToggle());
    SmartDashboard.putData(new ShoulderSetup());
    //SmartDashboard.putData(new TelescopeSetup());
    //SmartDashboard.putData(new WristSetup());
    SmartDashboard.putData(new SetArmBrake());
    SmartDashboard.putData(new SetArmCoast());

    /* Shoulder */
    setupShoulderChooser.addOption("Find Max/Mins", new ShoulderSetMaxMin());
    setupShoulderChooser.addOption("Export Motion Data", new ShoulderExportData());
    setupShoulderChooser.addOption("Move to 90", new ShoulderMoveLevel());
    setupShoulderChooser.addOption("Set Idle Pos", new ShoulderSetIdle());
    setupShoulderChooser.addOption("Set Level Pos", new ShoulderSetZero());
    setupShoulderChooser.setDefaultOption("None", new WaitCommand(1));
    SmartDashboard.putData(setupShoulderChooser);

    /* Wrist */
    // setupWristChooser.addOption("Find Max/Mins", new WristSetMaxMin()); // Read comment at top of class
    /*setupWristChooser.addOption("Set Max (90 deg)", new WristSetMax());
    setupWristChooser.addOption("Set Idle Pos", new WristSetIdle());
    setupWristChooser.addOption("Set Min (Level)", new WristSetMin());
    setupWristChooser.addOption("Export Motion Data", new WristExportData());
    setupWristChooser.setDefaultOption("None", new WaitCommand(1));
    SmartDashboard.putData(setupWristChooser);*/


    /* Telescope */
    /*setupTelescopeChooser.addOption("Set Min (In)", new TelescopeSetMin());
    setupTelescopeChooser.addOption("Set Max (Out)", new TelescopeSetMax());
    setupTelescopeChooser.setDefaultOption("None", new WaitCommand(1));
    SmartDashboard.putData(setupTelescopeChooser);*/

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
