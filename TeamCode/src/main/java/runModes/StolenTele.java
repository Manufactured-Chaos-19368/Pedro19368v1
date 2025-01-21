package runModes;


import static pedroPathing.constants.FieldConstants.intakePose;
import static pedroPathing.constants.RobotConstants.DRIVE_SPEED_FULL;
import static pedroPathing.constants.RobotConstants.DRIVE_SPEED_MEDIUM;
import static pedroPathing.constants.RobotConstants.DRIVE_SPEED_SLOW;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.constants.RobotConstants;
import subsystems.BucketSubsystem;
import subsystems.IntakeSubsystem;
import subsystems.LiftSubsystem;
import subsystems.LinkageSubsystem;

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



public class StolenTele {

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

    private Timer autoBucketTimer = new Timer(), transferTimer = new Timer(), bucketTimer = new Timer();

    private int flip = 1, autoBucketState = -1, transferState = -1, dumpBucketState = -1, adjustLift = 0;

    public double speed = 0.75;

    private boolean fieldCentric, actionBusy;

    private PathChain autoBucketTo, autoBucketBack;
    private Pose autoBucketToEndPose, autoBucketBackEndPose;

    public boolean highToggle = true;


    public StolenTele(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, Pose startPose,  boolean fieldCentric, Gamepad gamepad1, Gamepad gamepad2) {
        bucket = new BucketSubsystem(hardwareMap, bucketState, telemetry);
        lift = new LiftSubsystem(hardwareMap, telemetry);
        extend = new LinkageSubsystem(hardwareMap, linkageState, telemetry);
        intake = new IntakeSubsystem(hardwareMap, intakeSpinState, wristPivotState);

        Constants.setConstants(FConstants.class, LConstants.class);

        this.follower = follower;
        this.startPose = startPose;

        this.startPose = new Pose(-8, 36, Math.toRadians(0));

        this.fieldCentric = fieldCentric;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void init() {}

    public void start() {
        lift.start();
        extend.start();
        intake.start();
        bucket.toTransfer();
        follower.setPose(startPose);

    }

    public void update() {

        if (actionNotBusy()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            //****DRIVER 1 CORRECTED ACTIONS****\\

            // SPEED TOGGLE
            if (gamepad1.right_bumper) {
                speed = DRIVE_SPEED_MEDIUM;
            } else if (gamepad1.left_bumper) {
                speed = DRIVE_SPEED_SLOW;
            } else {
                speed = DRIVE_SPEED_FULL;
            }



            //****DRIVER 2 CORRECTED ACTIONS****\\

            /*
            square:         toggle high and low basket
            triangle:       set lift correction higher
            cross:          set lift correction lower

            right trigger:  extend linkage manually
            left trigger:   retract linkage manually

            left stick Y:   manual control lift

            DPAD Right:     toggle wrist position
            DPAD Left:      dump bucket
            DPAD Down:      move to pre intake position
            DPAD Up:        retract linkage and start transfer of sample

            left bumper:    intake sample
            right bumper:   spit out sample

             */

            if (currentGamepad2.square && !previousGamepad2.square) {
                highToggle = !highToggle;
            } else if(currentGamepad2.triangle && !previousGamepad2.triangle){
                adjustLift = adjustLift + 10;
            } else if(currentGamepad2.cross && !previousGamepad2.cross){
                adjustLift = adjustLift -10;
            }

            // TOUCH TRIGGERED TRANSFER
            if(intake.getIntakeTouch() && wristPivotState.equals(IntakeSubsystem.WristPivotState.OUT )){
                startTransfer();
            }

            // LINKAGE MANUAL CONTROL
            if (gamepad2.right_trigger > .1)
                extend.manual(1);
            else if (gamepad2.left_trigger > .1)
                extend.manual(-1);
            else
                extend.manual(0);

            // LIFT MANUAL CONTROL
            lift.manual(-currentGamepad2.left_stick_y);

            // WRIST PIVOT TOGGLE
            if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right){
                intake.switchPivotState();}

            // DUMP BUCKET AND RETRACT LIFT
            else if(currentGamepad2.dpad_left && !previousGamepad2.dpad_left && bucket.getBucketState() == BucketSubsystem.BucketState.MID){
                startDumpBucket();
            }

            //  MOVE OUT TO PRE INTAKE POSITION
            else if(currentGamepad2.dpad_down && !previousGamepad2.dpad_down && lift.getPos() < 1000){
                extend.toMid();
                intake.wristMid();
            }
            // MANUAL CONTROL OF TRANSFER OF SAMPLE
            else if(currentGamepad2.dpad_up && !previousGamepad2.dpad_up){
                startTransfer();
            }

            //INTAKE ON/OFF/REVERSE
            if (gamepad2.left_bumper)
                intake.setSpinState(IntakeSubsystem.IntakeSpinState.IN, false);
            else if (gamepad2.right_bumper)
                intake.setSpinState(IntakeSubsystem.IntakeSpinState.OUT, false);
            else
                intake.setSpinState(IntakeSubsystem.IntakeSpinState.STOP, false);

            //****UNCORRECTED ACTIONS****\\

            if(gamepad1.dpad_left)
                //startAutoBucket();

            if (gamepad1.x) {
               // flip = -1;
            }

            if (gamepad1.b) {
                //flip = 1;
            }

            // DRIVE STUFF
            follower.setTeleOpMovementVectors(flip * -gamepad1.left_stick_y * speed, flip * -gamepad1.left_stick_x * speed, -gamepad1.right_stick_x * speed * 0.5, !fieldCentric);
        } else {
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

        telemetry.addData("Action Busy", actionBusy);
        telemetry.addData("Auto Bucket State", autoBucketState);
        telemetry.update();
    }

    // CORRECTED ACTIONS

    public void dumpBucket(){
        switch (dumpBucketState) {
            case 1:
                bucket.toDeposit();
                setBucketState(2);
                break;

            case 2:
                if(bucketTimer.getElapsedTimeSeconds()>1){
                    bucket.toTransfer();
                    lift.toTransfer();
                    setBucketState(3);
                }
                break;

            case 3:
                actionBusy = false;
                setBucketState(-1);
        }
    }

    public void setBucketState(int x){
        dumpBucketState = x;
        bucketTimer.resetTimer();
    }

    public void startDumpBucket(){
        if(actionNotBusy()){
            setBucketState(1);
        }
    }

    public void transfer() {
        switch (transferState) {
            case 1:
                actionBusy = true;
                intake.spinStop();
                lift.toTransfer();  //test this with lift.toTransferCorrected(adjustLift)
                intake.wristIn();
                extend.toIn();
                setTransferState(2);
                break;
            case 2:
                if (transferTimer.getElapsedTimeSeconds() > .75) {
                    intake.spinOut();
                    setTransferState(3);
                }
                break;
            case 3:
                if (transferTimer.getElapsedTimeSeconds() > 1) {
                    if(highToggle){lift.toHighBasket();}
                    else{lift.toLowBasket();}
                    setTransferState(4);
                }
                break;
            case 4:
                if (transferTimer.getElapsedTimeSeconds() > 0.3) {
                    bucket.toMid();
                    intake.spinStop();
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


    private void intakePos() {
        bucket.toTransfer();
        lift.toTransfer();
        extend.toMid();
        intake.wristOut();
    }

    /*TODO AUTOBUCKET errored on me when tested it last time.  However I copied this over directly from
      TODO good luck testing.
    */
    private void autoBucket() {
        switch (autoBucketState) {
            case 1: //  MOVE TO BASKET AND RAISE LIFT AND BUCKET
                actionBusy = true;
                follower.breakFollowing();
                follower.setMaxPower(0.85);

                //Todo change the following pose to the pose we use for scoring the bucket
                autoBucketToEndPose = new Pose(-16.0, 2.5, Math.toRadians(45));

                autoBucketTo = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Point(follower.getPose()),
                                new Point(-16, 20.000, Point.CARTESIAN),
                                new Point(autoBucketToEndPose)))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), autoBucketToEndPose.getHeading())
                        .build();

                follower.followPath(autoBucketTo, true);

                if (autoBucketTimer.getElapsedTimeSeconds() > 1.5) {
                    lift.toHighBasket();
                }

                if (lift.getPos() > 500) {
                    bucket.toMid();
                }
                setAutoBucketState(2);
                break;

            case 2: // DUMP THE BASKET
                if (((follower.getPose().getX() > autoBucketToEndPose.getX() + 0.5) && //these may be wrong:)
                        (follower.getPose().getY() > autoBucketToEndPose.getY() - 0.5)) &&
                        (lift.getPos() > RobotConstants.LiftHighBasket - 50) &&
                        autoBucketTimer.getElapsedTimeSeconds() > 1) {
                    bucket.toDeposit();
                    setAutoBucketState(3);
                }
                break;

            case 3: // lower the lift and move back to submersible
                if (autoBucketTimer.getElapsedTimeSeconds() > 1) {
                    bucket.toTransfer();
                    lift.toTransfer();
                    autoBucketBackEndPose = new Pose(-5, 30, Math.toRadians(0));//will need to adjust this pose
                    autoBucketBack = follower.pathBuilder()
                            .addPath(new BezierCurve(new Point(follower.getPose()), new Point(58.000, 119.000, Point.CARTESIAN), new Point(autoBucketBackEndPose)))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), autoBucketToEndPose.getHeading())
                            .build();

                    follower.followPath(autoBucketBack, true);
                    setAutoBucketState(4);
                }
                if (autoBucketTimer.getElapsedTimeSeconds() > 2.5) {
                    extend.toMid();
                    intake.wristMid();
                    setAutoBucketState(4);
                }
                break;
            case 4:  //TODO start here ezra
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