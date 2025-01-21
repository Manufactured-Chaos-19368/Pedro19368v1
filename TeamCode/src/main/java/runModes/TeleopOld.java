package runModes;

import static pedroPathing.constants.FieldConstants.intakePose;
import static pedroPathing.constants.FieldConstants.parkPose;
import static pedroPathing.constants.FieldConstants.scoreControlPose;
import static pedroPathing.constants.FieldConstants.scorePose;
import static pedroPathing.constants.FieldConstants.titanStartPose;

import android.annotation.SuppressLint;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.constants.RobotConstants;
import subsystems.BucketSubsystem;
import subsystems.IntakeSubsystem;
import subsystems.LiftSubsystem;
import subsystems.LinkageSubsystem;


public class TeleopOld {
    public BucketSubsystem bucket;
    public BucketSubsystem.BucketState bucketState;
    public LiftSubsystem lift;
    public LinkageSubsystem extend;
    public LinkageSubsystem.LinkageState linkageState;
    public IntakeSubsystem intake;
    public IntakeSubsystem.IntakeSpinState intakeSpinState;
    public IntakeSubsystem.WristPivotState wristPivotState;

    private Follower follower;
    private Pose startPose;

    private Telemetry telemetry;

    private Gamepad gamepad1, gamepad2;
    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();

    private Timer autoBucketTimer = new Timer();

    private int flip = 1, autoBucketState = -1;

    public double speed = 0.75;

    private boolean fieldCentric, actionBusy;

    private PathChain autoBucketTo, autoBucketBack;
    private Pose autoBucketToEndPose, autoBucketBackEndPose, autoBucketStartPose;

    public TeleopOld(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, Pose startPose, boolean fieldCentric, Gamepad gamepad1, Gamepad gamepad2 ){
        bucket = new BucketSubsystem(hardwareMap, bucketState, telemetry);
        lift = new LiftSubsystem(hardwareMap, telemetry);
        extend = new LinkageSubsystem(hardwareMap, linkageState, telemetry);
        intake = new IntakeSubsystem(hardwareMap, intakeSpinState, wristPivotState);

        Constants.setConstants(FConstants.class, LConstants.class);

        this.follower = follower;
        this.telemetry = telemetry;

        //this.startPose = new Pose(56,102.25,Math.toRadians(270));
        this.startPose = parkPose;

        this.fieldCentric = fieldCentric;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void init() {}

    public void start(){
        lift.start();
        extend.start();
        intake.start();
        bucket.toTransfer();
        follower.setPose(titanStartPose);
        follower.startTeleopDrive();
    }

    @SuppressLint("SuspiciousIndentation")
    public void update(){
        if (actionNotBusy()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (gamepad1.right_bumper)
                speed = 1;
            else if (gamepad1.left_bumper)
                speed = 0.25;
            else
                speed = 0.75;


            if (gamepad1.x) {
                flip = -1;
            }

            if (gamepad1.b) {
                flip = 1;
            }

            if(gamepad1.dpad_left)
                autoBucketStartPose = follower.getPose();
                startAutoBucket();

            if(gamepad2.right_bumper)
                extend.manual(1);
            else if(gamepad2.left_bumper)
                extend.manual(-1);
            else
                extend.manual(0);
                follower.setTeleOpMovementVectors( -gamepad1.left_stick_y * speed,  -gamepad1.left_stick_x * speed, -gamepad1.right_stick_x * speed * 0.5, false);
        } else{
            if(gamepad1.dpad_right) {
                stopActions();
            }
        }

        lift.updatePIDF();

        autoBucket();

        follower.update();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addData("Lift Pos", lift.getPos());
        telemetry.addData("Extend Pos", extend.leftExtend.getPosition());
        telemetry.addData("Extend Limit", extend.extendLimit);
        telemetry.addData("Intake Spin State", intakeSpinState);
        telemetry.addData("Action Busy", actionBusy);
        telemetry.addData("Auto Bucket State", autoBucketState);
        telemetry.update();

    }

    private void autoBucket(){
        switch (autoBucketState){
            case 1: //  MOVE TO BASKET AND RAISE LIFT AND BUCKET
                actionBusy = true;
                follower.breakFollowing();
                follower.setMaxPower(0.85);

                autoBucketToEndPose = scorePose;

                autoBucketTo = follower.pathBuilder() //.correct scoreControlPose so we do not hit the submersible legs
                        .addPath(new BezierCurve(
                                new Point(follower.getPose()),
                                new Point(-16, 20.000, Point.CARTESIAN),
                                //new Point(scoreControlPose),
                                new Point(autoBucketBackEndPose)))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), autoBucketToEndPose.getHeading())
                        .build();

                follower.followPath(autoBucketTo, true);

                if(autoBucketTimer.getElapsedTimeSeconds() > 1.5){
                    lift.toHighBasket();
                }

                if(lift.getPos() > 500){
                    bucket.toMid();
                }
               // if(autoBucketTimer.getElapsedTimeSeconds() > 1.75){
                  //  bucket.toMid();
                //}
                setAutoBucketState(2);
                break;

            case 2: // DUMP THE BASKET
                if (((follower.getPose().getX() >  autoBucketToEndPose.getX() + 0.5) &&
                        (follower.getPose().getY() > autoBucketToEndPose.getY() - 0.5)) &&
                        (lift.getPos() > RobotConstants.LiftHighBasket - 50) &&
                        autoBucketTimer.getElapsedTimeSeconds() > 1) {
                   bucket.toDeposit();
                    setAutoBucketState(3);
                }

            break;

            case 3:  // lower the lift
                if(autoBucketTimer.getElapsedTimeSeconds()>1){
                    bucket.toTransfer();
                    lift.toTransfer();
                    autoBucketBackEndPose = intakePose;

                    autoBucketBack = follower.pathBuilder()
                            .addPath(new BezierCurve(new Point(follower.getPose()), new Point(scoreControlPose), new Point(autoBucketBackEndPose)))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), autoBucketToEndPose.getHeading())
                            .build();

                    follower.followPath(autoBucketBack, true);
                }

                if(autoBucketTimer.getElapsedTimeSeconds()>2.5){
                    extend.toMid();
                    intake.wristMid();
                    setAutoBucketState(4);
                }
                break;
            case 4: //close out the action
                follower.breakFollowing();
                follower.setMaxPower(1);
                follower.startTeleopDrive();
                actionBusy = false;
                setAutoBucketState(-1);
                break;
        }
    }

    public void setAutoBucketState(int x) {
        autoBucketState = x;
        autoBucketTimer.resetTimer();
    }

    public void startAutoBucket() {
        setAutoBucketState(1);
    }

    private boolean actionNotBusy() {
        return !actionBusy;
    }

    private void stopActions() {
        follower.breakFollowing();
        follower.setMaxPower(1);
        follower.startTeleopDrive();
        actionBusy = false;
        setAutoBucketState(-1);
    }

}
