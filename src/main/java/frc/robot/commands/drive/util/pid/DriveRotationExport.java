package frc.robot.commands.drive.util.pid;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.math.BigDecimal;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveRotationExport extends CommandBase {
    private BufferedWriter writer;
    private String swerveDumpFolder = "/home/lvuser/motion/swerve";
    private String swerveDumpFile = swerveDumpFolder+"/rotation_export.csv";
    private long time;

    double rotateFactorOne = Constants.MINIMUM_ROTATIONAL_OUTPUT;
    double rotateFactorTwo = Constants.MINIMUM_ROTATIONAL_OUTPUT*2;
    double inputSpeed = 0.0d;

    public DriveRotationExport() {
        time = -1L;
        inputSpeed = 0.0d;
        addRequirements(RobotContainer.swerveDrive);
    }
    
    @Override
    public void initialize() {
        RobotContainer.swerveDrive.stopAllModules();
        RobotContainer.swerveDrive.resetPose();
        RobotContainer.swerveDrive.resetContinuousRotPos(0);
        time = -1L;
        inputSpeed = 0.0d;
        setupFileWriting();
    }
  
    @Override
    public void execute() {
        // we kinda have to do a cluster of if statements because the file writing needs to be happening continuously 
        // aka i just forgot how threads work and im lazy lol
        if(time == -1L) {
            time = System.currentTimeMillis();
        }
        doSteps();
        writeData(inputSpeed, RobotContainer.swerveDrive.getSensorRotPos(0));
    }
  
    @Override
    public void end(boolean interrupted) {
        time = -1L;
        inputSpeed = 0.0d;
        try{
            writer.close();
        } catch(Exception exception) {
            System.out.println("DriveRotationExport: Error closing BufferedWriter object");
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
            RobotContainer.swerveDrive.simpleDriveRotationControlPercent(inputSpeed, 0);
        } else if(isInTimeRange(3.5d, 7.0d)) {
            inputSpeed = rotateFactorOne;
            RobotContainer.swerveDrive.simpleDriveRotationControlPercent(inputSpeed, 0);
        } else if(isInTimeRange(7.0d,10.5d)) {
            inputSpeed = 0.0d;
            RobotContainer.swerveDrive.simpleDriveRotationControlPercent(inputSpeed, 0);
        }
    }

    private void secondStep() {
        if(isInTimeRange(10.5d, 14.0d)) {
            inputSpeed = -rotateFactorOne;
            RobotContainer.swerveDrive.simpleDriveRotationControlPercent(inputSpeed, 0);
        } else if(isInTimeRange(14.0d, 17.5d)) {
            inputSpeed = 0.0d;
            RobotContainer.swerveDrive.simpleDriveRotationControlPercent(inputSpeed, 0);
        }
    }

    private void thirdStep() {
        if(isInTimeRange(17.5d, 21.0d)) {
            inputSpeed = rotateFactorTwo;
            RobotContainer.swerveDrive.simpleDriveRotationControlPercent(inputSpeed, 0);
        }
        if(isInTimeRange(21.0d, 24.5d)) {
            inputSpeed = 0.0d;
            RobotContainer.swerveDrive.simpleDriveRotationControlPercent(inputSpeed, 0);
        }
    }

    private void fourthStep() {
        if(isInTimeRange(24.5d, 28.0d)) {
            inputSpeed = -rotateFactorTwo;
            RobotContainer.swerveDrive.simpleDriveRotationControlPercent(inputSpeed, 0);
        }
        if(isInTimeRange(28.0d, 31.5d)) {
            inputSpeed = 0.0d;
            RobotContainer.swerveDrive.simpleDriveRotationControlPercent(inputSpeed, 0);
        } else if(isInTimeRange(31.5d, 100.0d)) {
            inputSpeed = 0.0d;
            this.end(true);
        }
    }

    private boolean isInTimeRange(double min, double max) {
        long difference = System.currentTimeMillis()-time;
        long minTermsMilli = (long) (min*1000L);
        long maxTermsMilli = (long) (max*1000L);
        return (difference >= minTermsMilli && difference < maxTermsMilli);
    }

    private void writeData(double input, double rotOut) {
        long difference = System.currentTimeMillis()-time;
        BigDecimal bd = BigDecimal.valueOf(difference).movePointLeft(3);
        try{
            writer.append(String.valueOf(bd.doubleValue()) + "," + String.valueOf(input) + "," + String.valueOf(rotOut) + "\n");
        } catch(Exception exception) {
            System.out.println("DriveRotationExport: Error writing motion data");
            exception.printStackTrace();
        }
    }

    private void setupFileWriting() {
        try { 
            new File(swerveDumpFolder).mkdir();
        } catch(Exception e){
            System.out.println("DriveRotationExport: Error creating directory");
            this.end(true);
            e.printStackTrace();
        }
        try { 
            new File(swerveDumpFile).createNewFile();
        } catch(Exception e){
            System.out.println("DriveRotationExport: Error creating file");
            this.end(true);
            e.printStackTrace();
        }
        try{
            writer = new BufferedWriter(new FileWriter(swerveDumpFile));
        } catch(Exception e){ 
            System.out.println("DriveRotationExport: Error initializing BufferedWriter");
            this.end(true);
            e.printStackTrace();
        }
        try { 
            writer.append("Time,Input,Output");
            writer.newLine();
        } catch(Exception e){
            System.out.println("DriveRotationExport: Error Writing CSV Header");
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
  
