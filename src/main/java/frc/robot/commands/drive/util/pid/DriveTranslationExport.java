package frc.robot.commands.drive.util.pid;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTranslationExport extends CommandBase {
    private BufferedWriter writer;
    private String swerveDumpFolder = "/home/lvuser/motion/swerve";
    private String swerveDumpFile = swerveDumpFolder+"/translation_export.csv";
    private long time;

    double driveFactorOne = Constants.MINIMUM_DRIVE_SPEED;
    double driveFactorTwo = Constants.MINIMUM_DRIVE_SPEED*2;
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
        if(isInTimeRange(0.0d, 3.5d)) {
            RobotContainer.swerveDrive.simpleDriveControlPercent(inputSpeed);
        } else if(isInTimeRange(3.5d, 7.0d)) {
            inputSpeed = driveFactorOne;
            RobotContainer.swerveDrive.simpleDriveControlPercent(inputSpeed);
        } else if(isInTimeRange(7.0d, 10.5d)) {
            inputSpeed = 0.0d;
            RobotContainer.swerveDrive.simpleDriveControlPercent(inputSpeed);
        }
    }

    private void secondStep() {
        if(isInTimeRange(10.5d, 14.0d)) {
            inputSpeed = -driveFactorOne;
            RobotContainer.swerveDrive.simpleDriveControlPercent(inputSpeed);
        } else if(isInTimeRange(14.0d, 17.5d)) {
            inputSpeed = 0.0d;
            RobotContainer.swerveDrive.simpleDriveControlPercent(inputSpeed);
        }
    }

    private void thirdStep() {
        if(isInTimeRange(17.5d, 21.0d)) {
            inputSpeed = driveFactorTwo;
            RobotContainer.swerveDrive.simpleDriveControlPercent(inputSpeed);
        } else if(isInTimeRange(21.0d, 24.5d)) {
            inputSpeed = 0.0d;
            RobotContainer.swerveDrive.simpleDriveControlPercent(inputSpeed);
        }
    }

    private void fourthStep() {
        if(isInTimeRange(24.5d, 28.0d)) {
            inputSpeed = -driveFactorTwo;
            RobotContainer.swerveDrive.simpleDriveControlPercent(inputSpeed);
        } else if(isInTimeRange(28.0d, 31.5d)) {
            inputSpeed = 0.0d;
            RobotContainer.swerveDrive.simpleDriveControlPercent(inputSpeed);
        }
    }

    private boolean isInTimeRange(double min, double max) {
        // the time field is in terms of time as milliseconds
        // min and max are in terms of seconds
        long difference = System.currentTimeMillis()-time;
        double diffTermsOfSecs = difference/1000;
        return (diffTermsOfSecs >= min && diffTermsOfSecs < max);
    }

    private void writeData(double input, double xOut) {
        long difference = System.currentTimeMillis()-time;
        try{
        writer.append(String.valueOf(difference) + "," + String.valueOf(input) + "," + String.valueOf(xOut));
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
  