package subsystems;

import static pedroPathing.constants.RobotConstants.LiftHighBasket;
import static pedroPathing.constants.RobotConstants.LiftLowBasket;
import static pedroPathing.constants.RobotConstants.LiftRest;
import static pedroPathing.constants.RobotConstants.LiftTransfer;
import static pedroPathing.constants.RobotConstants.LiftZero;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.arcrobotics.ftclib.controller.PIDController;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;


import util.action.RunAction;

public class LiftSubsystem {
    private Telemetry telemetry;

    public DcMotor rightLift, leftLift;
    public TouchSensor liftTouch;
    public boolean manual = false;
    public int pos, bottom;
    public RunAction toZero, toRest, toHighBasket, toLowBasket, toTransfer, toTransferCorrected;
    public PIDController liftPID;
    public static int target;
    public static double p = 0.01, i = 0, d = 0;
    public static double f = 0.005;

    public LiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightLift = hardwareMap.get(DcMotor.class, "LiftR");
        leftLift = hardwareMap.get(DcMotor.class, "LiftL");

        liftTouch = hardwareMap.get(TouchSensor.class, "LiftTouch");

        rightLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        liftPID = new PIDController(p, i, d);

        toZero = new RunAction(this::toZero);
        toRest = new RunAction(this::toRest);
        toHighBasket = new RunAction(this::toHighBasket);
        toLowBasket = new RunAction(this::toLowBasket);
        toTransfer = new RunAction(this::toTransfer);
    }


    public void updatePIDF(){
        if (!manual) {
            liftPID.setPID(p,i,d);

            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            double pid = liftPID.calculate(getPos(), target);
            double ticks_in_degrees = 384.5 / 360.0;
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
            double power = pid + ff;

            rightLift.setPower(power);
            leftLift.setPower(power);

            telemetry.addData("lift pos", getPos());
            telemetry.addData("lift target", target);
        }
    }

    public void manual(double n){
        manual = true;

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLift.setPower(n);
        leftLift.setPower(n);

        if(liftTouch.isPressed() & n < 0){
            rightLift.setPower(0);
            leftLift.setPower(0);

            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
    }

    //Util
    public void targetCurrent() {
        setTarget(getPos());
        manual = false;
    }

    public double getTarget() {
        return target;
    }

    public void setTarget(int b) {
        target = b;
    }

    public void addToTarget(int b) {
        target += b;
    }

    public int getPos() {
        pos = rightLift.getCurrentPosition() - bottom;
        return rightLift.getCurrentPosition() - bottom;
    }

    // OpMode
    public void init() {
        liftPID.setPID(p,i,d);
        bottom = getPos();
    }

    public void start() {
        target = 0;
    }

    //Presets

    public void toZero() {
        manual = false;
        setTarget(LiftZero);
    }

    public void toHighBasket() {
        manual = false;
        setTarget(LiftHighBasket);
    }

    public void toLowBasket() {
        setTarget(LiftLowBasket);
    }

    public void toTransfer() {
        manual = false;

        setTarget(LiftTransfer);
    }

    public void toTransferCorrected(int x) {
        manual = false;
        setTarget(LiftTransfer + x);
    }

    public void toRest() {
        manual = false;
        setTarget(LiftRest);
    }


}
