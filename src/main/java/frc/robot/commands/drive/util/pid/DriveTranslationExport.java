package frc.robot.commands.drive.util.pid;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.math.BigDecimal;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTranslationExport extends CommandBase {
    private BufferedWriter writer;
    private String swerveDumpFolder = "/home/lvuser/motion/swerve";
    private String swerveDumpFile = swerveDumpFolder+"/translation_export.csv";
    private long time;

    double driveFactorOne = Constants.MINIMUM_DRIVE_DUTY_CYCLE;
    double driveFactorTwo = Constants.MINIMUM_DRIVE_DUTY_CYCLE*2;
    double inputSpeed = 0.0d;

    public DriveTranslationExport() {
        time = -1L;
        inputSpeed = 0.0d;
        addRequirements(RobotContainer.swerveDrive);
    }
    
    @Override
    public void initialize() {
        RobotContainer.swerveDrive.stopAllModules();
        RobotContainer.swerveDrive.resetPose();
        time = -1L;
        inputSpeed = 0.0d;
        setupFileWriting();
    }
  
    @Override
    public void execute() {
        if(time == -1L) {
            time = System.currentTimeMillis();
        }
        // we kinda have to do a cluster of if statements because the file writing needs to be happening continuously 
        // aka i just forgot how threads work and im lazy lol
        doSteps();
        writeData(inputSpeed, RobotContainer.swerveDrive.getCurPose2d().getX());
    } 
  
    @Override
    public void end(boolean interrupted) {
        time = -1L;
        inputSpeed = 0.0d;
        try{
            writer.close();
        } catch(Exception exception) {
            System.out.println("DriveTranslationExport: Error closing BufferedWriter object");
            exception.printStackTrace();
        }
        RobotContainer.swerveDrive.stopAllModules();
    }

    private void doSteps() {
        firstStep();
        secondStep();
        thirdStep();
        fourthStep();
    }

    private void firstStep() {
        if(isInTimeRange(0.0d, 2d)) {
            RobotContainer.swerveDrive.simpleDriveControlPercent(inputSpeed);
        } else if(isInTimeRange(2, 4)) {
            inputSpeed = driveFactorOne;
            System.out.println(inputSpeed);
            RobotContainer.swerveDrive.simpleDriveControlPercent(inputSpeed);
        } else if(isInTimeRange(4, 6)) {
            inputSpeed = 0.0d;
            RobotContainer.swerveDrive.simpleDriveControlPercent(inputSpeed);
        }
    }

    private void secondStep() {
        if(isInTimeRange(6, 8)) {
            inputSpeed = -driveFactorOne;
            RobotContainer.swerveDrive.simpleDriveControlPercent(inputSpeed);
        } else if(isInTimeRange(8.0d, 10)) {
            inputSpeed = 0.0d;
            RobotContainer.swerveDrive.simpleDriveControlPercent(inputSpeed);
        }
    }

    private void thirdStep() {
        if(isInTimeRange(10, 12.0d)) {
            inputSpeed = driveFactorTwo;
            RobotContainer.swerveDrive.simpleDriveControlPercent(inputSpeed);
        } else if(isInTimeRange(12.0d, 14.0d)) {
            inputSpeed = 0.0d;
            RobotContainer.swerveDrive.simpleDriveControlPercent(inputSpeed);
        }
    }

    private void fourthStep() {
        if(isInTimeRange(14.0d, 16.0d)) {
            inputSpeed = -driveFactorTwo;
            RobotContainer.swerveDrive.simpleDriveControlPercent(inputSpeed);
        } else if(isInTimeRange(18.0d, 20.0d)) {
            inputSpeed = 0.0d;
            RobotContainer.swerveDrive.simpleDriveControlPercent(inputSpeed);
        } else if(isInTimeRange(20.0d, 100.0d)) {
            this.end(false);
        }
    }

    private boolean isInTimeRange(double min, double max) {
        long difference = System.currentTimeMillis()-time;
        long minTermsMilli = (long) (min*1000L);
        long maxTermsMilli = (long) (max*1000L);
        return (difference >= minTermsMilli && difference < maxTermsMilli);
    }

    private void writeData(double input, double xOut) {
        long difference = System.currentTimeMillis()-time;
        BigDecimal bd = BigDecimal.valueOf(difference).movePointLeft(3);
        try{
            writer.append(String.valueOf(bd.doubleValue()) + "," + String.valueOf(input) + "," + String.valueOf(xOut) + "\n");
        } catch(Exception exception) {
            System.out.println("DriveTranslationExport: Error writing motion data");
            exception.printStackTrace();
        }
    }

    private void setupFileWriting() {
        try { 
            new File(swerveDumpFolder).mkdir();
        } catch(Exception e){
            System.out.println("DriveTranslationExport: Error creating directory");
            this.end(true);
            e.printStackTrace();
        }
        try { 
            new File(swerveDumpFile).createNewFile();
        } catch(Exception e){
            System.out.println("DriveTranslationExport: Error creating file");
            this.end(true);
            e.printStackTrace();
        }
        try{
            writer = new BufferedWriter(new FileWriter(swerveDumpFile));
        } catch(Exception e){ 
            System.out.println("DriveTranslationExport: Error initializing BufferedWriter");
            this.end(true);
            e.printStackTrace();
        }
        try { 
            writer.append("Time,Input,Output");
            writer.newLine();
        } catch(Exception e){
            System.out.println("DriveTranslationExport: Error Writing CSV Header");
            this.end(true);
            e.printStackTrace();
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
  }
  