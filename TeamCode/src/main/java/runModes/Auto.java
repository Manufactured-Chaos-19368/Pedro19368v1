package runModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static pedroPathing.constants.FieldConstants.*;
import static pedroPathing.constants.FieldConstants.parkControlPose;
import static pedroPathing.constants.FieldConstants.parkPose;

import subsystems.BucketSubsystem;
import subsystems.IntakeSubsystem;
import subsystems.LiftSubsystem;
import subsystems.LinkageSubsystem;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class Auto {
    //private RobotStart startLocation;

    public BucketSubsystem bucket;
    public BucketSubsystem.BucketState bucketState;
    public LiftSubsystem lift;
    public LinkageSubsystem extend;
    public LinkageSubsystem.LinkageState linkageState;
    public IntakeSubsystem intake;
    public IntakeSubsystem.IntakeSpinState intakeSpinState;
    public IntakeSubsystem.WristPivotState wristPivotState;


    public Follower follower;
    public Telemetry telemetry;

    public boolean actionBusy, liftPIDF = true, intakeTouchToggle = false;
    public double liftManual = 0;

    public Timer transferTimer = new Timer(), preScoreTimer = new Timer(), dumpTimer = new Timer(), intakeTimer = new Timer(), parkTimer = new Timer(), postScoreTimer = new Timer(), chamberTimer2 = new Timer();
    public int transferState = -1, preScoreState = -1, intakeState = -1, parkState = -1, dumpState = -1, postScoreState = -1;

    //public Pose startPose, parkControlPose, parkPose;
    public Path scorePreload, park;
    public PathChain prePickup1, grabPickup1, farPickup1, scorePickup1, prePickup2, grabPickup2, farPickup2, scorePickup2, prePickup3, grabPickup3, scorePickup3;

    public Auto(HardwareMap hardwareMap, Telemetry telemetry, Follower follower) {
        bucket = new BucketSubsystem(hardwareMap, bucketState, telemetry);
        lift = new LiftSubsystem(hardwareMap, telemetry);
        extend = new LinkageSubsystem(hardwareMap, linkageState, telemetry);
        intake = new IntakeSubsystem(hardwareMap, intakeSpinState, wristPivotState);

        Constants.setConstants(FConstants.class, LConstants.class);

        this.follower = follower;
        this.telemetry = telemetry;

        buildPaths();

        init();
    }

    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        bucket.toTransfer();
        lift.init();
        extend.init();
        intake.init();
        follower.setStartingPose(titanStartPose);
        telemetryUpdate();
    }

    public void start() {
        Constants.setConstants(FConstants.class, LConstants.class);
        lift.start();
        extend.start();
        extend.start();
        intake.start();
        bucket.toTransfer();
        follower.setStartingPose(titanStartPose);
    }

    public void update() {
        follower.update();

        if(!liftPIDF)
            lift.manual(liftManual);
        else
            lift.updatePIDF();

        if(intakeTouchState())
            setIntakeTouchToggle(true);


        preScore();
        dump();
        postScore();
        transfer();
        intake();
        intakeTouchState();
        park();
        telemetryUpdate();
    }

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(titanStartPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(titanStartPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        prePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(rightSamplePreIntakePose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), rightSamplePreIntakePose.getHeading())
                .build();

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(rightSamplePreIntakePose), new Point(rightSampleIntakePose)))
                .setLinearHeadingInterpolation(rightSamplePreIntakePose.getHeading(), rightSampleIntakePose.getHeading())
                .build();

        farPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(rightSampleIntakePose), new Point(rightSampleIntakePoseFar)))
                .setLinearHeadingInterpolation(rightSampleIntakePose.getHeading(), rightSampleIntakePoseFar.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(rightSampleIntakePose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(rightSampleIntakePose.getHeading(), scorePose2.getHeading())
                .build();

        prePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(midSamplePreIntakePose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), midSamplePreIntakePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(midSamplePreIntakePose), new Point(midSampleIntakePose)))
                .setLinearHeadingInterpolation(midSamplePreIntakePose.getHeading(), midSampleIntakePose.getHeading())
                .build();

        farPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(midSampleIntakePose), new Point(midSampleIntakePoseFar)))
                .setLinearHeadingInterpolation(midSampleIntakePose.getHeading(), midSampleIntakePoseFar.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(midSampleIntakePose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(midSampleIntakePose.getHeading(), scorePose2.getHeading())
                .build();

        prePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(leftSamplePreIntakePose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leftSamplePreIntakePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(leftSamplePreIntakePose), new Point(leftSampleIntakePose)))
                .setLinearHeadingInterpolation(leftSamplePreIntakePose.getHeading(), leftSampleIntakePose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(leftSampleIntakePose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(leftSampleIntakePose.getHeading(), scorePose2.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose2), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose2.getHeading(), parkPose.getHeading());
    }

    public void preScore() {
        switch (preScoreState) {
            case 1:
                actionBusy = true;
                intake.spinStop();
                lift.toHighBasket();
                preScoreTimer.resetTimer();
                setPreScoreState(2);
                break;
            case 2:
                if (preScoreTimer.getElapsedTimeSeconds() > 0.25) {
                    bucket.toMid();
                    preScoreTimer.resetTimer();
                    setPreScoreState(3);
                }
                break;
            case 3:
                if (preScoreTimer.getElapsedTimeSeconds() > 1) {
                    actionBusy = false;
                    setPreScoreState(-1);
                }
                break;
        }
    }

    public void setPreScoreState(int x) {
        preScoreState = x;
    }

    public void startPreScore() {
        if (actionNotBusy()) {
            setPreScoreState(1);
        }
    }


    public void dump(){
        switch (dumpState){
            case 1:
                actionBusy = true;
                bucket.toDeposit();
                dumpTimer.resetTimer();
                setDumpState(2);
                break;
            case 2:
                if (dumpTimer.getElapsedTimeSeconds() > 1) { //change from 1.0 on 1/23
                    actionBusy = false;
                    setDumpState(-1);
                }
                break;
        }
    }

    public void setDumpState(int x){
        dumpState = x;
    }

    public void startDump(){
        if (actionNotBusy()) {
            setDumpState(1);
        }
    }


    public void postScore(){
        switch (postScoreState){
            case 1:
                actionBusy = true;
                bucket.toTransfer();
                lift.toTransfer();
                extend.toMid();
                intake.wristOut();
                postScoreTimer.resetTimer();
                setPostScoreState(2);
                break;
            case 2:
                if (preScoreTimer.getElapsedTimeSeconds() > 0.5) {
                    intake.spinIn();
                    actionBusy = false;
                    setPostScoreState(-1);
                }
                break;
        }
    }

    public void setPostScoreState(int x){
        postScoreState  = x;
    }

    public void startPostScore(){
        if (actionNotBusy()) {
            setPostScoreState(1);
        }
    }


    public void transfer() {
        switch (transferState) {
            case 1:
                actionBusy = true;
                intake.spinStop();
                lift.toTransfer();
                intake.wristIn();
                extend.toIn();
                setTransferState(2);
                break;
            case 2:
                if (transferTimer.getElapsedTimeSeconds() > 1.25) { //changed from 1.25 on 1/23
                    intake.spinOut();
                    setTransferState(3);
                }
                break;
            case 3:
                if (transferTimer.getElapsedTimeSeconds() > 1) {
                    lift.toHighBasket();
                    setTransferState(4);
                }
                break;
            case 4:
                if (transferTimer.getElapsedTimeSeconds() > 0.3) {
                    bucket.toMid();
                    setTransferState(5);
                }
            case 5:
                if (transferTimer.getElapsedTimeSeconds() > 0.2) {
                    actionBusy = false;
                    setTransferState(-1);
                }

                    break;
        }
    }

    public void setTransferState(int x) {
        transferState = x;
        transferTimer.resetTimer();

    }

    public void startTransfer() {
        if (actionNotBusy()) {
            setTransferState(1);
        }
    }


    public void intake() {
        switch (intakeState) {
            case 1:
                actionBusy = true;
                //claw.open();
                intakeTimer.resetTimer();
                setTransferState(2);
                break;
            case 2:
                if(intakeTimer.getElapsedTimeSeconds() > 0.5) {
                    //arm.transfer();
                    //claw.transfer();
                    //intake.pivotTransfer();
                    intake.spinStop();
                    lift.toTransfer();
                    //claw.open();
                    //extend.toHalf();
                    intakeTimer.resetTimer();
                    setTransferState(3);
                }
                break;
            case 3:
                if (intakeTimer.getElapsedTimeSeconds() > 0.5) {
                    //intake.pivotGround();
                    intake.spinIn();
                    intakeTimer.resetTimer();
                    setTransferState(4);
                }
                break;
            case 4:
                if (intakeTimer.getElapsedTimeSeconds() > 1) {
                    intake.spinStop();
                    intakeTimer.resetTimer();
                    actionBusy = false;
                    setTransferState(-1);
                }
                break;
        }
    }

    public void setIntakeState(int x) {
        intakeState = x;
    }

    public void startIntake() {
        if (actionNotBusy()) {
            setIntakeState(1);
        }
    }

    public boolean intakeTouchState(){
        return intake.getIntakeTouch();
    }

    public void setIntakeTouchToggle(boolean x){
        intakeTouchToggle = x;
    }

    public boolean getIntakeTouchToggle(){
        return intakeTouchToggle;
    }


    public void park() {
        switch (parkState) {
            case 1:
                actionBusy = true;
                parkTimer.resetTimer();
                setParkState(2);
                break;
            case 2:
                if(parkTimer.getElapsedTimeSeconds() > 0.25) {
                    intake.wristIn();
                    intake.spinStop();
                    bucket.toTransfer();
                    lift.toZero();
                    extend.toIn();
                    parkTimer.resetTimer();
                    actionBusy = false;
                    setTransferState(-1);
                }
                break;
        }
    }

    public void setParkState(int x) {
        parkState = x;
    }

    public void startPark() {
        if (actionNotBusy()) {
            setParkState(1);
        }
    }


    public boolean actionNotBusy() {
        return !actionBusy;
    }

    public boolean notBusy() {
        return (!follower.isBusy() && actionNotBusy());
    }

    public void telemetryUpdate() {
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.addData("Action Busy?: ", actionBusy);
        telemetry.addData("Lift Pos", lift.getPos());
        telemetry.addData("Extend Pos", extend.leftExtend.getPosition());
        telemetry.update();
    }
}
