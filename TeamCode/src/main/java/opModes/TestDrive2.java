package opModes;

import static pedroPathing.constants.FieldConstants.parkControlPose;
import static pedroPathing.constants.FieldConstants.parkPose;
import static pedroPathing.constants.FieldConstants.scorePose;
import static pedroPathing.constants.FieldConstants.scorePose2;
import static pedroPathing.constants.RobotConstants.DRIVE_SPEED_FULL;
import static pedroPathing.constants.RobotConstants.DRIVE_SPEED_MEDIUM;
import static pedroPathing.constants.RobotConstants.DRIVE_SPEED_SLOW;
import static pedroPathing.constants.RobotConstants.fastSpeed;
import static pedroPathing.constants.RobotConstants.slowSpeed;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import subsystems.BucketSubsystem;
import subsystems.IntakeSubsystem;
import subsystems.LiftSubsystem;
import subsystems.LinkageSubsystem;

/**
 * This is an example teleop that showcases movement and field-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */
@Disabled
@TeleOp(name = "Test Drive 2", group = "Examples")
public class TestDrive2 extends OpMode {
    //public Gamepad gamepad1;
    public Gamepad currentGamepad1, previousGamepad1, currentGamepad2, previousGamepad2;

    private final Timer autoBucketTimer = new Timer();
    private final Timer transferTimer = new Timer();
    private final Timer bucketTimer = new Timer();

    public boolean actionBusy, liftPIDF = true, highToggle = true;

    private int autoBucketState = -1;
    private int transferState = -1;
    private int dumpBucketState = -1;
    private int adjustLift = 0;

    public double liftManual = 0;
    public double lateralDrive;

    private Follower follower;
    private final Pose startPose = parkPose;//new Pose(0,0,0);

    private boolean driveTypeFieldCentric = false;
    public double speed;
    public BucketSubsystem bucket;
    public BucketSubsystem.BucketState bucketState;
    public LiftSubsystem lift;
    public LinkageSubsystem extend;
    public LinkageSubsystem.LinkageState linkageState;
    public IntakeSubsystem intake;
    public IntakeSubsystem.IntakeSpinState intakeSpinState;
    public IntakeSubsystem.WristPivotState wristPivotState;

    private PathChain autoScore;
    private Path scoreSample, returnToSubmersable, scorePickup3;

    /**
     * This method is call once when init is played, it initializes the follower
     **/
    @Override
    public void init() {

        //Gamepad currentGamepad1 =  new Gamepad();
        //Gamepad previousGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();

        bucket = new BucketSubsystem(hardwareMap, bucketState, telemetry);
        lift = new LiftSubsystem(hardwareMap, telemetry);
        extend = new LinkageSubsystem(hardwareMap, linkageState, telemetry);
        intake = new IntakeSubsystem(hardwareMap, intakeSpinState, wristPivotState);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setHeadingOffset(-90);

        follower.setStartingPose(startPose);

        lift.init();
    }

    /**
     * This method is called continuously after Init while waiting to be started.
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     **/
    @Override
    public void start() {
        follower.startTeleopDrive();
        lift.start();
        extend.start();
        intake.start();
        bucket.toTransfer();
    }

    /**
     * This is the main loop of the opmode and runs continuously after play
     **/
    @Override
    public void loop() {

        //call all actions that have state controls
        transfer();
        autoBucket();
        bail();
        startTouchTransfer();
        dumpBucket();

        // GAMEPAD TRACKING
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (!liftPIDF) {
            lift.manual(liftManual);
        }else{
            lift.updatePIDF();}

        //lift.updatePIDF();

        //****DRIVER 1 CORRECTED ACTIONS****\\
        if (currentGamepad1.cross && !previousGamepad1.cross) {
            driveTypeFieldCentric = !driveTypeFieldCentric;
        }

        //todo test this code after driver 2 testing.  Remove if it does not work.
        if(currentGamepad1.triangle && !previousGamepad1.triangle){
            startAutoBucket();
        }

        if(currentGamepad1.circle && !previousGamepad1.circle){
            stopActions();
        }

        // SPEED TOGGLE
        if (gamepad1.right_bumper) {
            speed = DRIVE_SPEED_MEDIUM;
        } else if (gamepad1.left_bumper) {
            speed = DRIVE_SPEED_SLOW;
        } else {
            speed = DRIVE_SPEED_FULL;
        }

        //Dpad straffing
        if (gamepad1.dpad_right) {
            lateralDrive = -.5;
            driveTypeFieldCentric = true;
        } else if (gamepad1.dpad_left) {
            lateralDrive = .5;
            driveTypeFieldCentric = true;

        } else {
            lateralDrive = -gamepad1.left_stick_x;
            driveTypeFieldCentric = false;
        }

        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y * speed,
                lateralDrive * speed,
                -gamepad1.right_stick_x * speed,
                driveTypeFieldCentric);

        follower.update();

        //DRIVER 2***************************************************************
         /*
            '-' indicates that it has been tested and works
            '*' indicates that it has not been tested
            '?' indicates that is is not working

            *square:         toggle high and low basket
            *triangle:       set lift correction higher
            *cross:          set lift correction lower

            -right trigger:  extend linkage manually: this works
            -left trigger:   retract linkage manually: this works

            ?left stick Y:   manual control lift: not working very well-retest

            -DPAD Right:     toggle wrist position: this works
            -DPAD Left:      dump bucket & return slide & bucket down: this works
            -DPAD Down:      move to pre intake position: this works
            ?DPAD Up:        retract linkage and start transfer of sample: not spitting the sample into bucket

            -left bumper:    intake sample
            -right bumper:   spit out sample

            ?touch sensor:   not triggering the action.  need to test again with this updated action.

            *touchpad:        bail

             */

        // BUTTON CONTROLS
        if (currentGamepad2.square && !previousGamepad2.square) {
            highToggle = !highToggle;
        } else if (currentGamepad2.triangle && !previousGamepad2.triangle) {
            adjustLift = adjustLift + 10;
        } else if (currentGamepad2.cross && !previousGamepad2.cross) {
            adjustLift = adjustLift - 10;
        }

        // LINKAGE MANUAL CONTROL
        if (gamepad2.right_trigger > .1)
            extend.manual(1);
        else if (gamepad2.left_trigger > .1)
            extend.manual(-1);
        else
            extend.manual(0);

        // DPAD CONTROLS
        if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
            intake.switchPivotState();
        }
        if (gamepad2.dpad_down) {
            intakePos();
        } else if (gamepad2.dpad_left) {
            startDumpBucket();
        } else if (gamepad2.dpad_up) {
            startTransfer();
        }

        //INTAKE ON/OFF/REVERSE
        if (gamepad2.left_bumper) {
            intake.setSpinState(IntakeSubsystem.IntakeSpinState.IN, false);
        }else if (gamepad2.right_bumper) {
            intake.setSpinState(IntakeSubsystem.IntakeSpinState.OUT, false);
        }else if( !gamepad2.left_bumper && !gamepad2.right_bumper && !actionBusy){
            intake.setSpinState(IntakeSubsystem.IntakeSpinState.STOP, false);
        }

        // MANUAL LIFT CONTROL
        lift.manual(-currentGamepad2.left_stick_y);

        /* Telemetry Outputs of our Follower */
        telemetry.addData("Lift Adjust", adjustLift);
        telemetry.addData("Intake Touch", intake.getIntakeTouch());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    /**
     * We do not use this because everything automatically should disable
     **/
    @Override
    public void stop() {
    }


    public void dumpBucket() {
        switch (dumpBucketState) {
            case 1:
                actionBusy = true;
                liftPIDF = true;
                bail();
                bucket.toDeposit();
                setBucketState(2);
                break;

            case 2:
                if (bucketTimer.getElapsedTimeSeconds() > 1) {
                    bail();
                    bucket.toTransfer();
                    lift.toTransferCorrected(adjustLift);
                    setBucketState(3);
                }
                break;

            case 3:
                actionBusy = false;
                setBucketState(-1);
        }
    }

    public void setBucketState(int x) {
        dumpBucketState = x;
        bucketTimer.resetTimer();
    }

    public void startDumpBucket() {
        if (actionNotBusy()) {
            setBucketState(1);
        }
    }

    public void transfer() {
        switch (transferState) {
            case 1:
                actionBusy = true;
                liftPIDF = true;
                intake.spinStop();
                bucket.toTransfer();
                lift.toTransferCorrected(adjustLift);
                intake.wristIn();
                extend.toIn();
                bail();
                setTransferState(2);
                break;
            case 2: //give time for all the previous actions to complete
                if (transferTimer.getElapsedTimeSeconds() > .5) {
                    bail();
                    setTransferState(21);
                }
                break;
            case 21: //start spitting out the sample
                intake.spinOut();
                setTransferState(22);
                break;
            case 22: //spit for 1 second then stop
                if(transferTimer.getElapsedTimeSeconds() >1){
                    bail();
                    setTransferState(3);
                }
                break;
            case 3: //raise to either high or low basket
                if (highToggle) {
                    intake.spinStop();
                    lift.toHighBasket();
                    bail();
                    setTransferState(4);
                } else {
                    intake.spinStop();
                    lift.toLowBasket();
                    bail();
                    setTransferState(4);
                }
                break;
            case 4:
                if (transferTimer.getElapsedTimeSeconds() > 0.3) {
                    bail();
                    bucket.toMid();
                    setTransferState(5);
                }
                break;
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

    public void startTouchTransfer(){
        if(intake.getIntakeTouch()){
            startTransfer();
        }
    }


    private void intakePos() {
        bucket.toTransfer();
        lift.toTransferCorrected(adjustLift);

        extend.toMid();
        intake.wristMid();
    }


    private boolean actionNotBusy() {
        return !actionBusy;
    }

    private void stopActions() {
        follower.breakFollowing();
        follower.setMaxPower(speed);
        follower.startTeleopDrive();
        actionBusy = false;
        //setAutoBucketState(-1);
    }


    private void bail() {
        if (gamepad2.touchpad) {
            setTransferState(-1);
            setBucketState(-1);
            bucket.toTransfer();
            lift.toRest();
            intake.wristIn();
            intake.spinStop();
            extend.toIn();

        }
    }

    private void autoBucket() {
        /*
        We have rotated the heading in teleop for use in fieldcentric drive.  This will probably
        mess with the heading of the auto movements during this action.
        You may have to rotate the heading back and to start and end the action.
        */

        switch (autoBucketState) {
            case 1: //  create our path chains
                actionBusy = true;
                buildPaths();

                //follower.breakFollowing();
                follower.setMaxPower(slowSpeed);
                follower.followPath(scoreSample, true);

                setAutoBucketState(2);
                break;

            case 2:
                if(!follower.isBusy() && actionNotBusy()) {
                    //startDumpBucket();
                    setAutoBucketState(3);
                    if(autoBucketTimer.getElapsedTimeSeconds() > .5){
                        follower.setMaxPower(fastSpeed);
                        follower.followPath(returnToSubmersable);
                    }
                    setAutoBucketState(3);
                }
                break;
            case 3:
                if(!follower.isBusy() && actionNotBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setAutoBucketState(-1);
                }
                break;
        }
    }

    public void setAutoBucketState(int x) {
        autoBucketState = x;
        autoBucketTimer.resetTimer();
    }

    public void startAutoBucket() {
        if (actionNotBusy()) {
            setAutoBucketState(1);
        }
    }


    public void buildPaths() {
        scoreSample = new Path(new BezierCurve(
                new Point(follower.getPose()), /* Control Point */
                new Point(parkControlPose),
                new Point(scorePose2)));
        scoreSample.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());

        returnToSubmersable = new Path(new BezierCurve(
                new Point(scorePose), /* Control Point */
                new Point(parkControlPose),
                new Point(parkPose)));
        returnToSubmersable.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }
}
