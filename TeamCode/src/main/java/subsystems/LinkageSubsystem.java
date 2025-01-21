package subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static pedroPathing.constants.RobotConstants.LLinkageIn;
import static pedroPathing.constants.RobotConstants.LLinkageMax;
import static pedroPathing.constants.RobotConstants.LLinkagePartial;
import static pedroPathing.constants.RobotConstants.LinkageManualIncrements;
import static pedroPathing.constants.RobotConstants.RLinkageIn;
import static pedroPathing.constants.RobotConstants.RLinkageMax;
import static pedroPathing.constants.RobotConstants.RLinkagePartial;

import util.action.RunAction;

public class LinkageSubsystem {
    private Telemetry telemetry;

    public enum LinkageState{
        TRANSFER, MID, MAX
    }

    private LinkageState linkageState;

    public Servo leftExtend, rightExtend;
    private double lPos = 0;
    public double extendLimit = LLinkageMax;
    public RunAction toIn, toMid, toFull;

    public LinkageSubsystem(HardwareMap hardwareMap, LinkageState linkageState, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.linkageState = linkageState;

        leftExtend = hardwareMap.get(Servo.class, "Leftlinkage");
        rightExtend = hardwareMap.get(Servo.class, "Rightlinkage");

        leftExtend.setDirection(Servo.Direction.REVERSE);

        toIn = new RunAction(this::toIn);
        toMid = new RunAction(this::toMid);
        toFull = new RunAction(this::toFull);
    }

    public void manual(int direction) {
        double leftPos = leftExtend.getPosition();
        double rightPos = rightExtend.getPosition();

        if (leftPos <= extendLimit || direction < 0) {
            leftPos += (LinkageManualIncrements * direction);
            rightPos += (LinkageManualIncrements * direction);
        } else {
            leftPos = rightExtend.getPosition();
            rightPos = rightExtend.getPosition();
        }

        leftExtend.setPosition(rightPos);
        rightExtend.setPosition(rightPos);
    }

    public void setTarget(double left, double right) {
        leftExtend.setPosition(left);
        rightExtend.setPosition(right);
        lPos = left;
    }

    public void toIn() {
        setTarget(LLinkageIn, RLinkageIn);
        this.linkageState = LinkageState.TRANSFER;

    }

    public void toMid() {
        setTarget(LLinkagePartial, RLinkagePartial);
        this.linkageState = LinkageState.MID;
    }

    public void toFull() {
        setTarget(LLinkageMax, RLinkageMax);
        this.linkageState = LinkageState.MAX;
    }

    public LinkageState getLinkageState() {
        return linkageState;
    }

    // Util //
    public double getPos() {
        updatePos();
        return lPos;
    }

    public void updatePos() {
        lPos = leftExtend.getPosition();
    }

    // Init + Start //
    public void init() {
        updatePos();
        toIn();
    }

    public void start() {
        updatePos();
        toIn();
    }
}
