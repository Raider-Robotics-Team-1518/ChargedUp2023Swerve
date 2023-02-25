package frc.robot.subsystems.moving;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    // "shoulder" is the pivot point at the top of the tower, raises & lowers the arm
    private final CANSparkMax shoulderMotor = new CANSparkMax(Constants.ARM_SHOULDER_ID, MotorType.kBrushless);
    // "wrist" is the pivot point at the end of the arm, raises and lowers the claw
    private final CANSparkMax wristMotor = new CANSparkMax(Constants.ARM_WRIST_ID, MotorType.kBrushless);
    // "telescope" is the part of the arm that extends/retracts
    private final CANSparkMax telescopeMotor = new CANSparkMax(Constants.ARM_TELESCOPE_ID, MotorType.kBrushless);

    public final PIDController shoulderPidController = new PIDController(Constants.ARM_SHOULDER_P, Constants.ARM_SHOULDER_I, Constants.ARM_SHOULDER_D);
    public final PIDController wristPidController = new PIDController(Constants.ARM_WRIST_P, Constants.ARM_WRIST_I, Constants.ARM_WRIST_D);


    /*
     * Moving Parts
     * !!!! IMPORTANT: SETUP of these VARIABLES using the designated SETUP COMMANDS is REQUIRED before BOT USAGE !!!!!
     */

    /*
     * Steps:
     * 1. Set Shoulder Level position using ShoulderSetZero
     * 2. Set Shoulder Maximum and Minimum lists using ShoulderSetMaxMin
     * 3. 
     */

    // Shoulder
    public DigitalInput shoulderLowerSwitch = new DigitalInput(0);
    public DigitalInput shoulderUpperSwitch = new DigitalInput(1);
    public double maxShoulderPos = Preferences.getDouble(Constants.SHOULDER_MAX_POS, 50d); // Arm is facing the ceiling (180 deg)
    public double horizontalShoulderPos = Preferences.getDouble(Constants.SHOULDER_LEVEL_POS, 0d); // 90 deg
    public double minShoulderPos = Preferences.getDouble(Constants.SHOULDER_MIN_POS, -50d); // As retracted as we can get without breaking electronics (unknown angle)
    public double idleShoulderAngle = Preferences.getDouble(Constants.SHOULDER_IDLE_ANGLE, 145d); // (degrees)

    // Wrist
    public double maxWristPos = Preferences.getDouble(Constants.WRIST_MAX_POS, 50d); // 180 deg
    public double minWristPos = Preferences.getDouble(Constants.WRIST_MIN_POS, -50d); // 0 deg
    public double idleWristAngle = Preferences.getDouble(Constants.WRIST_IDLE_ANGLE, 180d); // (degrees)
    public boolean lockedWrist = true;

    // Telescope
    public double fullyWoundTelescopePositon = Preferences.getDouble(Constants.TELESCOPE_MIN_POS, 0d); // Telescope fully retracted
    public double fullyUnwoundTelescopePositon = Preferences.getDouble(Constants.TELESCOPE_MAX_POS, 50d); // Telescope fully extended


    // Objects specific to the dump process
    private boolean isDumping = false;
    private boolean doneDumping = false;
    private DumpMode currentDumpMode = DumpMode.SHOULDER;
    private String shoulderDumpFile = "/home/lvuser/1518dump/shoulder_pid_data.csv";
    private String wristDumpFile = "/home/lvuser/1518dump/wirst_pid_data.csv";
    private BufferedWriter writer;
    private long start = -1L;

    public static ArmSubsystem INSTANCE;
    public ArmSubsystem() {
        INSTANCE = this;
        try { 
            new File("/home/lvuser/1518dump").mkdir();
            new File(shoulderDumpFile).createNewFile();
            new File(wristDumpFile).createNewFile();
            writer = new BufferedWriter(new FileWriter(shoulderDumpFile)); 
        } catch(Exception e){ 
            e.printStackTrace(); 
        }
        start = -1L;

        shoulderMotor.setIdleMode(IdleMode.kBrake);

        // shoulder pid setup
        setShoulderTargetPos(idleShoulderAngle, true);
        shoulderPidController.setTolerance(0.1);

        // wrist pid setup
        setWristTargetPos(idleWristAngle, true);
        wristPidController.setTolerance(0.1);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ShoulderEncVal", getShoulderPosition());
        SmartDashboard.putNumber("ShoulderDesiredPos", shoulderPidController.getSetpoint());
        SmartDashboard.putNumber("WristEncVal", getWristPosition());

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

        if(!DriverStation.isEnabled() || Constants.setupState) return;
        /*!!!!!!!!!!!!!!!!! Enable-Require Tasks !!!!!!!!!!!!!!!!!!!!*/
        //fixateShoulder(); fixate just shoulder motor
        //fixateWrist(); fixate just wrist motor
        fixateArm(true); // fixate both
    }

    public void setWristTargetPos(double target, boolean angle) {
        double targetPos = target;
        if(angle) targetPos = ((target/180)*maxWristPos);
        wristPidController.setSetpoint(targetPos);
    }

    public void setShoulderTargetPos(double target, boolean angle) {
        double targetPos = target;
        if(angle) {
            if(target > 90) {
                targetPos = ((target-90)/90)*maxShoulderPos;
            } else {
                targetPos = -(maxShoulderPos+((-(target/90))*maxShoulderPos));
            }
        }
        shoulderPidController.setSetpoint(targetPos);
    }

    public void fixateArm(boolean lockedWrist) {
        if(lockedWrist) {
            setWristTargetPos(90+getShoulderAngle(), true);
        }
        fixateShoulder();
    }

    public void fixateWrist() {
        if(!wristPidController.atSetpoint()) {
            wristMotor.set(wristPidController.calculate(this.getWristPosition(), wristPidController.getSetpoint()));
        } else {
            wristMotor.set(0);
        }
    }

    public void fixateShoulder() {
        if(!shoulderPidController.atSetpoint()) {
            shoulderMotor.set(shoulderPidController.calculate(this.getShoulderPosition(), shoulderPidController.getSetpoint()));
        } else {
            shoulderMotor.set(0);
        }
    }

    public boolean isWristInRange() {
        double wristPos = wristMotor.getEncoder().getPosition();
        return ((wristPos < maxWristPos) && (wristPos > minWristPos));
    }

    public boolean isTelescopeInRange() {
        double telescopePos = telescopeMotor.getEncoder().getPosition();
        return ((telescopePos < fullyWoundTelescopePositon) && (telescopePos > fullyUnwoundTelescopePositon));
    }

    public double getWristAngle() {
        return ((getWristPosition()/maxWristPos)*180);
    }


    public double getShoulderAngle() {
        /* since i only know the position from (zero/90 deg) horizontal to (180 deg) the highest point, 
        calculations must be done to account for negative values */
        if(getShoulderPosition() > 0) {
            // above (zero) horizontal pos
            return 90+((Math.abs(getShoulderPosition()) / maxShoulderPos)*90);
        }
        return 90-((Math.abs(getShoulderPosition()) / maxShoulderPos)*90);
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

    public void toggleDumping(DumpMode dumpMode) {
        INSTANCE.currentDumpMode = dumpMode;
        INSTANCE.isDumping = !INSTANCE.isDumping;
    }

    public void setShoulderMaxPos(double highestPosition) {
        INSTANCE.maxShoulderPos = highestPosition;
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
