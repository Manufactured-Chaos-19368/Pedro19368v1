package subsystems;

import static pedroPathing.constants.RobotConstants.WristIn;
import static pedroPathing.constants.RobotConstants.WristMid;
import static pedroPathing.constants.RobotConstants.WristOut;
import static pedroPathing.constants.RobotConstants.intakeSpinInPwr;
import static pedroPathing.constants.RobotConstants.intakeSpinOutPwr;
import static pedroPathing.constants.RobotConstants.intakeSpinStopPwr;

import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import util.action.RunAction;

public class IntakeSubsystem {

    public enum IntakeSpinState{
        IN, OUT, STOP
    }

    public enum WristPivotState{
        IN, MID, OUT
    }

    public CRServo Lintake, Rintake;
    private IntakeSpinState spinState;

    public Servo RWrist, LWrist;
    public WristPivotState wristState;

    public TouchSensor Touch;

    public RunAction spinIn, spinOut, spinStop, wristIn, wristMid, wristOut;

    public IntakeSubsystem(HardwareMap hardwareMap, IntakeSpinState spinState, WristPivotState wristState) {
        this.spinState = spinState;
        this.wristState = wristState;

        RWrist = hardwareMap.get(Servo.class, "Rreach");
        LWrist = hardwareMap.get(Servo.class, "Lreach");
        Rintake = hardwareMap.get(CRServo.class, "Rintake");
        Lintake = hardwareMap.get(CRServo.class, "Lintake");
        Touch = hardwareMap.get(TouchSensor.class, "Touch");

        Rintake.setDirection(CRServo.Direction.REVERSE);
        LWrist.setDirection(Servo.Direction.REVERSE);

        spinIn = new RunAction(this::spinIn);
        spinOut = new RunAction(this::spinOut);
        spinStop = new RunAction(this::spinStop);
        wristIn = new RunAction(this::wristIn);
        wristMid = new RunAction(this::wristMid);
        wristOut = new RunAction(this::wristOut);
    }

    // ----------------- Intake Spin -----------------//

    public void setSpinState(IntakeSpinState spinState, boolean changeStateOnly) {
        if (changeStateOnly) {
            this.spinState = spinState;
        } else {
            if (spinState == IntakeSpinState.IN) {
                spinIn();
            } else if (spinState == IntakeSpinState.OUT) {
                spinOut();
            } else if (spinState == IntakeSpinState.STOP) {
                spinStop();
            }
        }
    }

    public void spinIn() {
        Lintake.setPower(intakeSpinInPwr);
        Rintake.setPower(intakeSpinInPwr);
        this.spinState = IntakeSpinState.IN;
    }

    public void spinInBackAlways() {
        Lintake.setPower(intakeSpinInPwr);
        Rintake.setPower(intakeSpinInPwr);
        this.spinState = IntakeSpinState.IN;
    }

    public void spinOut() {
        Lintake.setPower(intakeSpinOutPwr);
        Rintake.setPower(intakeSpinOutPwr);
        this.spinState = IntakeSpinState.OUT;
    }

    public void spinStop() {
        Lintake.setPower(intakeSpinStopPwr);
        Rintake.setPower(intakeSpinStopPwr);
        this.spinState = IntakeSpinState.STOP;
    }

    // ----------------- Intake Pivot -----------------//

    public void setPivotState(WristPivotState pivotState) {
        if (pivotState == WristPivotState.IN) {
            LWrist.setPosition(WristIn);
            RWrist.setPosition(WristIn);
            this.wristState = WristPivotState.IN;
        } else if (pivotState == WristPivotState.MID) {
            LWrist.setPosition(WristMid);
            RWrist.setPosition(WristMid);
            this.wristState = WristPivotState.MID;
        } else if (pivotState == WristPivotState.OUT) {
            LWrist.setPosition(WristOut);
            RWrist.setPosition(WristOut);
            this.wristState = WristPivotState.OUT;
        }
    }

    public void switchPivotState() {
        if (wristState == WristPivotState.MID) {
            wristOut();
        } else if (wristState == WristPivotState.OUT) {
            wristMid();
        }
    }

    public void wristIn() {
        LWrist.setPosition(WristIn);
        RWrist.setPosition(WristIn);
        this.wristState = WristPivotState.IN;
    }

    public void wristMid() {
        LWrist.setPosition(WristMid);
        RWrist.setPosition(WristMid);
        this.wristState = WristPivotState.MID;
    }

    public void wristOut() {
        LWrist.setPosition(WristOut);
        RWrist.setPosition(WristOut);
        this.wristState = WristPivotState.OUT;
    }

    public boolean getIntakeTouch(){
        return Touch.isPressed();
    }


    public void init() {
        wristIn();
        spinStop();
    }

    public void start() {
        wristIn();
        spinStop();
    }
}
