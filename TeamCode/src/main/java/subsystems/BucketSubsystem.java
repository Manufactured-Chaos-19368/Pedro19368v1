package subsystems;

import static pedroPathing.constants.RobotConstants.BucketDeposit;
import static pedroPathing.constants.RobotConstants.BucketMid;
import static pedroPathing.constants.RobotConstants.BucketTransfer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import util.action.RunAction;

public class BucketSubsystem {
    private Telemetry telemetry;

    public Servo bucket;

    public enum BucketState{
        TRANSFER, MID, DUMP
    }

    private BucketState bucketState;


    public RunAction toTransfer, toMid, toDeposit;

    public BucketSubsystem(HardwareMap hardwareMap, BucketState bucketState, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.bucketState = bucketState;

        bucket = hardwareMap.get(Servo.class, "Bucket");
        bucket.setDirection(Servo.Direction.REVERSE);

        toTransfer = new RunAction(this::toTransfer);
        toMid = new RunAction(this::toMid);
        toDeposit = new RunAction(this::toDeposit);
    }

    public void toTransfer(){
        bucket.setPosition(BucketTransfer);
        this.bucketState = BucketState.TRANSFER;
    }

    public void toMid(){
        bucket.setPosition(BucketMid);
        this.bucketState = BucketState.MID;
    }

    public void toDeposit(){
        bucket.setPosition(BucketDeposit);
        this.bucketState = BucketState.DUMP;
    }

    public BucketState getBucketState() {
        return bucketState;
    }
}
