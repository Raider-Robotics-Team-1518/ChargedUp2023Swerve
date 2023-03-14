package frc.robot.subsystems.moving;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    /*
     * Moving Parts
     * !!!! IMPORTANT: SETUP of these VARIABLES using the designated SETUP COMMANDS is REQUIRED before BOT USAGE !!!!!
     */

    // Shoulder
    private final CANSparkMax shoulderMotor = new CANSparkMax(Constants.ARM_SHOULDER_ID, MotorType.kBrushless);
    public final PIDController shoulderPidController = new PIDController(Constants.ARM_SHOULDER_P, Constants.ARM_SHOULDER_I, Constants.ARM_SHOULDER_D);
    public double maxShoulderPos = Constants.getDouble(Constants.SHOULDER_MAX_POS); // Arm is facing the ceiling (180 deg)
    public double horizontalShoulderPos = Constants.getDouble(Constants.SHOULDER_LEVEL_POS); // 90 deg
    public double idleShoulderAngle = Constants.getDouble(Constants.SHOULDER_IDLE_ANGLE); // (degrees)

    // Wrist
    private final CANSparkMax wristMotor = new CANSparkMax(Constants.ARM_WRIST_ID, MotorType.kBrushless);
    public final PIDController wristPidController = new PIDController(Constants.ARM_WRIST_P, Constants.ARM_WRIST_I, Constants.ARM_WRIST_D);
    public double maxWristPos = Constants.getDouble(Constants.WRIST_MAX_POS); // 90 deg (Facing up)
    public double minWristPos = Constants.getDouble(Constants.WRIST_MIN_POS); // 0 deg (Level)
    public double idleWristAngle = Constants.getDouble(Constants.WRIST_IDLE_ANGLE); // (degrees)
    public boolean lockedWrist = true;

    // Telescope
    private final CANSparkMax telescopeMotor = new CANSparkMax(Constants.ARM_TELESCOPE_ID, MotorType.kBrushless);
    public double minTelescopePos = Constants.getDouble(Constants.TELESCOPE_MIN_POS); // Telescope fully retracted
    public double maxTelescopePos = Constants.getDouble(Constants.TELESCOPE_MAX_POS); // Telescope fully extended

    // Objects specific to the dump process
    private boolean isDumping = false;
    private boolean doneDumping = false;
    private DumpMode currentDumpMode = DumpMode.WRIST;
    public String shoulderDumpFile = "/home/lvuser/motion/shoulder_motion_data.csv";
    public String wristDumpFile = "/home/lvuser/motion/wirst_motion_data.csv";
    public BufferedWriter writer;
    private long start = -1L;
    private double currentShoulderSetpoint;

    public static ArmSubsystem INSTANCE;
    public ArmSubsystem() {
        INSTANCE = this;

        setupDumping();
        setArmBrake();

        // shoulder pid setup
        currentShoulderSetpoint = 15d;
        setShoulderTargetPos(15, true);
        shoulderPidController.setTolerance(0.1);

        // wrist pid setup
        setWristTargetPos(idleWristAngle, true);
        wristPidController.setTolerance(0.1);

        wristMotor.setInverted(true);
    }

    @Override
    public void periodic() {

        // PID Data dumping
        if(isDumping) {
            try {
                if(start == -1L) {
                    start = System.currentTimeMillis();
                    writer.append("Time,Input,Output");
                    writer.newLine();
                }
                switch(currentDumpMode) {
                    
                    case SHOULDER:
                        doShoulderDump();
                        break;
                    case WRIST:
                        doWristDump();
                        break;
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        // SetupState Dashboard Information
        if(Constants.setupState) {
            SmartDashboard.putBoolean("ShlderFwdLimit", getShoulderMotor().getForwardLimitSwitch(Type.kNormallyOpen).isPressed());
            SmartDashboard.putBoolean("ShlderRevLimit", getShoulderMotor().getReverseLimitSwitch(Type.kNormallyOpen).isPressed());
            SmartDashboard.putNumber("ShoulderEncoderPos", getShoulderPosition());
            SmartDashboard.putNumber("ShoulderAngle", getShoulderAngle());
            SmartDashboard.putNumber("WristEncoderPos", getWristPosition());
            SmartDashboard.putNumber("WristAngle", getWristAngle());
            SmartDashboard.putNumber("TelescopePos", getTelescopePosition());
            return;
        }

        if(!DriverStation.isEnabled()) {
            return;
        }
        //!!!!!!!!!!!!!!!!! Enable-Require Tasks !!!!!!!!!!!!!!!!!!!!
        fixateShoulder(); //fixate just shoulder motor
        //fixateWrist();
        //fixateArm(INSTANCE.lockedWrist); // fixate both
    }

    public void offsetShoulder(double offset) {
        if(offset > 0) {
            // going up limit
            if(getShoulderMotor().getForwardLimitSwitch(Type.kNormallyOpen).isPressed()) offset = 0.0d;
        } else {
            // going down limit
            if(getShoulderMotor().getReverseLimitSwitch(Type.kNormallyOpen).isPressed()) offset = 0.0d;
        }
        currentShoulderSetpoint = currentShoulderSetpoint+offset;
    }

    public void setArmCoast() {
        shoulderMotor.setIdleMode(IdleMode.kCoast);
        wristMotor.setIdleMode(IdleMode.kCoast);
        telescopeMotor.setIdleMode(IdleMode.kBrake); // Should never have to leave brake mode for telescope
    }

    public void setArmBrake() {
        shoulderMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setIdleMode(IdleMode.kBrake);
        telescopeMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setTelescopeSpeed(double speed) {
        telescopeMotor.set(speed);
    }

    public void setupDumping() {
        try { 
            new File("/home/lvuser/motion").mkdir();
            new File(shoulderDumpFile).createNewFile();
            new File(wristDumpFile).createNewFile();
            writer = new BufferedWriter(new FileWriter(shoulderDumpFile)); 
        } catch(Exception e){ 
            e.printStackTrace(); 
        }
        start = -1L;

    }

    public double potentialWristTargetPos(double target, boolean angle) {
        double targetPos = target;
        if(angle) targetPos = ((target/90)*maxWristPos);
        return targetPos;
    }

    public void setWristTargetPos(double target, boolean angle) {
        double targetPos = target;
        if(angle) targetPos = ((target/90)*maxWristPos);
        wristPidController.setSetpoint(targetPos);
    }

    public void setShoulderTargetPos(double target, boolean angle) {
        double targetPos = target;
        if(angle) targetPos = ((target/180)*maxShoulderPos);
        currentShoulderSetpoint = targetPos;
    }

    public void fixateArm() {
        fixateWrist();
        fixateShoulder();
    }

    public void fixateWrist() {
        if(lockedWrist) {
            setWristTargetPos(90-getShoulderAngle(), true);
        }
        if(!wristPidController.atSetpoint()) {
            wristMotor.set(wristPidController.calculate(this.getWristPosition(), wristPidController.getSetpoint()));
        } else {
            wristMotor.set(0);
        }
    }

    public void fixateShoulder() {
        double motorOut = shoulderPidController.calculate(getShoulderPosition(), currentShoulderSetpoint);
        if(!shoulderPidController.atSetpoint()) {
            shoulderMotor.set(motorOut);
        } else {
            shoulderMotor.set(0);
        }
    }

    public boolean isWristInRange() {
        return true;
        //double wristPos = wristMotor.getEncoder().getPosition();
        //return ((wristPos < maxWristPos) && (wristPos > minWristPos));
    }

    public boolean canTelescopeExtend() {
        double telescopePos = Math.abs(telescopeMotor.getEncoder().getPosition());
        return (telescopePos <= maxTelescopePos);
    }

    public boolean canTelescopeRetract() {
        double telescopePos = Math.abs(telescopeMotor.getEncoder().getPosition());
        return (telescopePos >= minTelescopePos);
    }

    public boolean canEnableTelescope(boolean forward) {
        if(forward) {
            return canTelescopeExtend();
        }
        return canTelescopeRetract();
    }

    public boolean isTelescopeInRange(boolean forward) {
        double telescopePos = Math.abs(telescopeMotor.getEncoder().getPosition());
        if(forward) {

        }
        return ((telescopePos < maxTelescopePos) && (telescopePos > minTelescopePos));
    }

    public void telescopeExtend() {
        telescopeMotor.set(Constants.telescopeSpeed);
    }

    public void telescopeRetract() {
        telescopeMotor.set(-Constants.telescopeSpeed);
    }

    public boolean telescopeExtended() {
        return (getTelescopePosition() <= (maxTelescopePos-5));
    }

    public boolean telescopeRetracted() {
        return (getTelescopePosition() >= (minTelescopePos+5));
    }

    public double getWristAngle() {
        return ((getWristPosition()/maxWristPos)*180);
    }


    public double getShoulderAngle() {
        // max shoulder pos = (1/((Constants.ARM_SHOULDER_UPPERSWITCH_DEG+Constants.ARM_SHOULDER_LOWERSWITCH_DEG)/180))*upperEncVal;
        return ((getShoulderPosition()/maxShoulderPos)*180)+Constants.ARM_SHOULDER_LOWERSWITCH_DEG;
    }

    public void resetWristPosition() {
        wristMotor.getEncoder().setPosition(0.0d);
    }

    public double getWristPosition() {
        return wristMotor.getEncoder().getPosition();
    }


    public double getTelescopePosition() {
        return telescopeMotor.getEncoder().getPosition();
    }

    public CommandBase stopShoulder() {
        return runOnce(() -> shoulderMotor.set(0.0d));
    }

    public CommandBase stopWrist() {
        return runOnce(() -> wristMotor.set(0.0d));
    }

    public CommandBase stopTelescope() {
        return runOnce(() -> telescopeMotor.set(0.0d));
    }

    public CommandBase stopArm() {
        return Commands.sequence(stopShoulder(), stopWrist(), stopTelescope());
    }

    public void setWristSpeed(double speed) {
        wristMotor.set(speed);
    }

    public void toggleDumping(DumpMode dumpMode) throws IOException {
        switch(dumpMode) {
            case WRIST:
                writer = new BufferedWriter(new FileWriter(shoulderDumpFile)); 
                break;
            case SHOULDER:
                writer = new BufferedWriter(new FileWriter(shoulderDumpFile)); 
                break;
        }
        INSTANCE.currentDumpMode = dumpMode;
        INSTANCE.isDumping = !INSTANCE.isDumping;
    }

    public void setShoulderMaxPos(double highestPosition) {
        INSTANCE.maxShoulderPos = highestPosition;
    }

    public void resetTelescopePosition() {
        telescopeMotor.getEncoder().setPosition(0.0);
    }

    public void resetShoulderPosition() {
        shoulderMotor.getEncoder().setPosition(0.0);
    }

    public double getShoulderPosition() {
        return shoulderMotor.getEncoder().getPosition();
    }

    public void doShoulderDump() throws IOException {
        float timeElapsed = (System.currentTimeMillis() - start) ; // time elapsed in seconds
        if(timeElapsed >= 1000 && timeElapsed <= 2000) {
            shoulderMotor.set(0.1);
        } else if(timeElapsed > 2000 && timeElapsed <= 3000) {
            shoulderMotor.set(0);
        } else if(timeElapsed > 3000 && timeElapsed <= 3500) {
            shoulderMotor.set(-0.1);
        } else if(timeElapsed > 3500 && timeElapsed <= 4500) {
            shoulderMotor.set(0);
        } else if(timeElapsed > 4500 && timeElapsed <= 5500) {
            shoulderMotor.set(0.2);
        } else if(timeElapsed > 5500 && timeElapsed <= 6500) {
            shoulderMotor.set(0);
        } else if(timeElapsed > 6500 && timeElapsed <= 7000) {
            shoulderMotor.set(-0.2);
        } else if(timeElapsed > 7000 && timeElapsed <= 8000) {
            shoulderMotor.set(0.0);
        }

        writer.append(timeElapsed + "," + shoulderMotor.get() + "," + getShoulderPosition());
        writer.newLine();
        if(timeElapsed >= 8000) {
            isDumping = false;
            doneDumping = true;
            start = -1L;
            writer.close();
        }
    }

    public void doWristDump() throws IOException {
        float timeElapsed = (System.currentTimeMillis() - start) ; // time elapsed in seconds
        if(timeElapsed >= 1000 && timeElapsed <= 2000) {
            wristMotor.set(0.2);
        } else if(timeElapsed > 2000 && timeElapsed <= 3000) {
            wristMotor.set(0);
        } else if(timeElapsed > 3000 && timeElapsed <= 3500) {
            wristMotor.set(-0.2);
        } else if(timeElapsed > 3500 && timeElapsed <= 4500) {
            wristMotor.set(0);
        } else if(timeElapsed > 4500 && timeElapsed <= 5500) {
            wristMotor.set(0.4);
        } else if(timeElapsed > 5500 && timeElapsed <= 6500) {
            wristMotor.set(0);
        } else if(timeElapsed > 6500 && timeElapsed <= 7000) {
            wristMotor.set(-0.4);
        } else if(timeElapsed > 7000 && timeElapsed <= 8000) {
            wristMotor.set(0.0);
        }

        writer.append(timeElapsed + "," + shoulderMotor.get() + "," + getWristPosition());
        writer.newLine();
        if(timeElapsed >= 8000) {
            isDumping = false;
            doneDumping = true;
            start = -1L;
            writer.close();
        }
    }

    public boolean getDumping() {
        return INSTANCE.isDumping;
    }

    public void setDoneDumping(boolean b) {
        INSTANCE.doneDumping = b;
    }
    
    public boolean getDoneDumping() {
        return INSTANCE.doneDumping;
    }

    public CANSparkMax getShoulderMotor() {
        return INSTANCE.shoulderMotor;
    }

    public CANSparkMax getWristMotor() {
        return INSTANCE.wristMotor;
    }

    public CANSparkMax getTelescopeMotor() {
        return INSTANCE.telescopeMotor;
    }

    public static enum DumpMode {
        SHOULDER, WRIST
    }

}
