package opModes.opVault;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import util.action.GoBildaPinpointDriver;

import java.util.Locale;
@Deprecated
@Disabled
//Call the teleop so it shows up on the driver station.
@TeleOp(name = "Competition DriveOLD", group = "TeleOp")
public class CompDriveOLD extends LinearOpMode {
    //can have variables and define hardware objects here, anything from here to "waitForStart();" will run in initialization.\

    //Actuators
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private DcMotor LiftR;
    private DcMotor LiftL;

    private Servo Bucket;
    private Servo RWrist;
    private Servo LWrist;

    private Servo Leftlinkage;
    private Servo Rightlinkage;

    private CRServo Lintake;
    private CRServo Rintake;

    private TouchSensor Touch, Ltouch;

    //Timer variables
    double IntakeTime, IntakeTime2, IntakeTime3, IntakeTime4;

    //Gyros
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double Lift_PPR_To_Inches = 0.012;
    double inches = 0;

    //Drive speed variables .
    double DRIVE_SPEED_FULL = .90;
    final double DRIVE_SPEED_SLOW = .4;
    final double DRIVE_SPEED_MEDIUM = 0.60;
    double driveSpeed = DRIVE_SPEED_FULL;
    int headingOffset = 90;


    //Variables to hold motor powers
    double denominator;
    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;

    //Toggles
    private double toggle = -5;
    private boolean startup = true;

    //****Linkage****\\
    static public double LLinkageInPos = 0.245;
    static public double RLinkageInPos = 0.03;
    static public double LLinkagePartialPos = 0.37-.05; //3
    static public double RLinkagePartialPos = 0.151-.05; //
    static public double LLinkageMaxPos = 0.495;
    static public double RLinkageMaxPos = 0.272;

    //****Bucket****\\
    static public double BucketPreScorePos = 0.4;
    static public double BucketScorePos = 0.7;
    static public double BucketIntakePos = 0.0;

    //****Lift****\\
    double liftSpeed = 1;
    static public int LiftRestPos = 20;
    static public int LiftIntakePos = 300;//290
    static public int LiftDepositPos = 2300;
    static public int LiftLowBasketPos = 800;//26.5
    //static public int LiftHalfWayPos = 1000;
    static public int liftAdjust = 0;
    static public int liftAdjustCount = 0;

    //****Wrist****\\
    static public double LWristInPos = 0.018;
    static public double RWristInPos = 0.018;
    static public double LWristPartialPos = 0.373;
    static public double RWristPartialPos = 0.373;
    static public double LWristOutPos = 0.555;
    static public double RWristOutPos = 0.555;
    //static public double WristCorrection = .0;

    //****Rotate****\\
    //static public double hPivotCenterPos = .5;
    // static public double hPivotMaxPos = .84;
    //static public double hPivotMinPos = .16;

    static public boolean button_State = false;
    static public boolean down = true;

    //PID LiftController Stuff
    public static final double kP = .003;
    public static final double kI = .12;
    public static final double kD = .053;
    public static final double kThreshold = 8;



    public boolean highToggle = true;

    //start of opmode, inside this function will be your main while loop and initialize all hardware objects
    @Override
    public void runOpMode() {

        //Advanced gamepad control for driver 2
        Gamepad driver2Current = new Gamepad();
        Gamepad driver2Previous = new Gamepad();

        //Intialize Pinpoint ODO
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(22, -101);//in mm
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        //initialize motors, you will need to change these parameters to match your motor setup and names.

        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        LiftR = hardwareMap.get(DcMotor.class, "LiftR");
        LiftL = hardwareMap.get(DcMotor.class, "LiftL");
        Bucket = hardwareMap.get(Servo.class, "Bucket");
        RWrist = hardwareMap.get(Servo.class, "Rreach");
        LWrist = hardwareMap.get(Servo.class, "Lreach");
        Leftlinkage = hardwareMap.get(Servo.class, "Leftlinkage");
        Rightlinkage = hardwareMap.get(Servo.class, "Rightlinkage");
        Rintake = hardwareMap.get(CRServo.class, "Rintake");
        Lintake = hardwareMap.get(CRServo.class, "Lintake");
        Touch = hardwareMap.get(TouchSensor.class, "Touch");
        Ltouch = hardwareMap.get(TouchSensor.class, "LiftTouch");

        //change the braking behavior, this is mostly personal preference but I recommend leaving this unchanged.
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reverse motors
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        LiftL.setDirection(DcMotor.Direction.REVERSE);

// Value to ensure that the toggle is off
        toggle = -5;
        // The hands in and out rotation

        //LiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*LiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
        LiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rintake.setDirection(CRServo.Direction.REVERSE);
        Bucket.setDirection(Servo.Direction.REVERSE);
        LWrist.setDirection(Servo.Direction.REVERSE);
        Leftlinkage.setDirection(Servo.Direction.REVERSE);

        //wait for the driver station to start
        waitForStart();

        //primary while loop to call your various functions during driver control from
        while (opModeIsActive() && !isStopRequested()) {

            //Do servo stuff we cant do in initialization.
            if (startup) {
                //Step 1:   Rotate and lock hand in intake position
                //MoveHandRotation(hPivotCenterPos);

                //Step 2:
                //A:        Lock the wrist in position
                //B:        Extend the linkages
                //C:        Position the bucket
                //D:        Return linkages
                MoveWrist(RWristInPos, LWristInPos);
                //MoveLinkage(RLinkageInPos+.05, LLinkageInPos+.05);
                MoveBucket(BucketIntakePos);
                MoveLinkage(RLinkageInPos, LLinkageInPos);
                startup = false;
            }
            //Gamepad stuff
            driver2Previous.copy(driver2Current);
            driver2Current.copy(gamepad2);

            //Update odo every loop

//*************************************************************************
//DRIVER ONE CODE**********************************************************
//*************************************************************************

            //feed stick values into variables
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            //Update the odo each cycle so we have the most current data.
            odo.update();

            // We need to be able to correct the IMU if it drifts or is wrong.
            if (gamepad1.touchpad) { //need to set this to the correct button combo
                headingOffset = 0;
                odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }

            //gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
            //We can delete the following telemetry if we dont see a need for it.
            Pose2D pos = odo.getPosition(); //DO NOT DELETE THIS LINE! It is used later.
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));

            //gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
            Pose2D vel = odo.getVelocity();
            String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));

            //Get the bots heading from pinpoint for use with fieldcentric drive.
            double botHeading = pos.getHeading(AngleUnit.RADIANS) - Math.toRadians(headingOffset);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing


            if (gamepad1.right_bumper) {
                driveSpeed = DRIVE_SPEED_MEDIUM;
            } else if (gamepad1.left_bumper) {
                driveSpeed = DRIVE_SPEED_SLOW;
            } else {
                driveSpeed = DRIVE_SPEED_FULL;
            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            if (Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_x) > 0.1) {
                denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                frontLeftPower = (rotY + rotX + rx) / denominator;
                backLeftPower = (rotY - rotX + rx) / denominator;
                frontRightPower = (rotY - rotX - rx) / denominator;
                backRightPower = (rotY + rotX - rx) / denominator;

            } else if (gamepad1.dpad_down) {
                frontLeftPower = -1;
                backLeftPower = -1;
                frontRightPower = -1;
                backRightPower = -1;

            } else if (gamepad1.dpad_up) {
                frontLeftPower = 1;
                backLeftPower = 1;
                frontRightPower = 1;
                backRightPower = 1;

            } else if (gamepad1.dpad_left) {
                frontLeftPower = -1;
                backLeftPower = 1;
                frontRightPower = 1;
                backRightPower = -1;

            } else if (gamepad1.dpad_right) {
                frontLeftPower = 1;
                backLeftPower = -1;
                frontRightPower = -1;
                backRightPower = 1;

            } else if (gamepad1.right_trigger > .5) {  //Slow strafe right
                frontLeftPower = .3;
                backLeftPower = -.3;
                frontRightPower = -.3;
                backRightPower = .3;

            } else if (gamepad1.left_trigger > .5) {  //Slow strafe left
                frontLeftPower = -.3;
                backLeftPower = .3;
                frontRightPower = .3;
                backRightPower = -.3;

            } else {
                frontLeftPower = 0;
                backLeftPower = 0;
                frontRightPower = 0;
                backRightPower = 0;
            }


            //Set Power to drive motors
            FL.setPower(frontLeftPower * driveSpeed);
            BL.setPower(backLeftPower * driveSpeed);
            FR.setPower(frontRightPower * driveSpeed);
            BR.setPower(backRightPower * driveSpeed);

//*************************************************************************
//DRIVER TWO***************************************************************
//*************************************************************************

            // HIGH AND LOW BASKET TOGGLE
            if (driver2Current.square && !driver2Previous.square) {
                highToggle = !highToggle;
            }

            if (gamepad2.dpad_down && toggle == -5) {
                Bail();
                MoveLinkage(RLinkagePartialPos, LLinkagePartialPos);
                MoveWrist(RWristPartialPos, LWristPartialPos);
                toggle = -1;

            } else if (gamepad2.dpad_right && toggle == -1) {
                Bail();
                toggle = 0;

            } else if (toggle == 0) {
                Bail();
                // rotates the hand down to intake position
                // add a spot where the wheels only intake if told to.
                MoveWrist(RWristOutPos-.002 , LWristOutPos-.002 );


                if (toggle == 0 && gamepad2.right_trigger > .1) {
                    // Move linkage forwards manually
                    if (Leftlinkage.getPosition() < LLinkageMaxPos) {
                        MoveLinkage(Rightlinkage.getPosition() + .004, Leftlinkage.getPosition() + .004);
                    }

                }
                if (toggle == 0 && gamepad2.left_trigger > .1) {
                    // Move linkage backwards manually
                    if (Leftlinkage.getPosition() > LLinkageInPos) {
                        MoveLinkage(Rightlinkage.getPosition() - .004, Leftlinkage.getPosition() - .004);
                    }

                }
                if (gamepad2.left_bumper && toggle == 0) {
                    Bail();
                    MoveIntake(1);

                } else if (gamepad2.right_bumper && toggle == 0) {
                    Bail();
                    MoveIntake(-1);

                } else if (!gamepad2.right_bumper && !gamepad2.left_bumper && toggle == 0) {
                    Bail();
                    MoveIntake(0);
                }

                // returns bucket to a mid point to make sure it goes the correct way around
                if (Touch.isPressed()) {
                    Bail();
                    // checks if we have something intaken
                    toggle = 1;
                }

//RETURN MECH TO HOME POSITION AND PLACE SAMPLE IN BUCKET
            } else if (toggle == 1) {
                Bail();
                MoveBucket(BucketIntakePos);
                MoveLift(LiftIntakePos + liftAdjust, .5);
                MoveWrist(RWristInPos, LWristInPos);
                MoveLinkage(RLinkageInPos, LLinkageInPos);
                toggle = 2;
                IntakeTime = getRuntime();

            } else if (false) {
                toggle = 2;

                // HAND MECH IS BACK AND WE ARE GOING TURN ON THE INTAKE TO SPIT OUT THE SAMPLE
            } else if (toggle == 2 && getRuntime() >= IntakeTime + 1) {
                Bail();
                MoveIntake(-1);
                IntakeTime = getRuntime();
                toggle = 3;

//  AFTER A PAUSE RAISE THE LIFT TO DEPOSIT POSITION
            } else if (toggle == 3 && getRuntime() >= IntakeTime + .65) {
                Bail();

                if (highToggle) {
                    MoveLift(LiftDepositPos, liftSpeed);
                } else if (!highToggle) {
                    MoveLift(LiftLowBasketPos, liftSpeed);
                }

                IntakeTime = getRuntime();
                toggle = 4;

//  AFTER A PAUSE  MOVE THE BUCKET AND TURN OFF THE INTAKE
            } else if (toggle == 4 && getRuntime() >= IntakeTime + 0.25) {
                Bail();
                MoveIntake(0);
                MoveBucket(.35);
                toggle = 4.5;
            }

// DPAD LEFT TO DEPOSIT THE SAMPLE
            else if (toggle == 4.5/*5*/ && gamepad2.dpad_left) {
                Bail();
                MoveBucket(BucketScorePos);
                IntakeTime = getRuntime();
                toggle = 6;

                // AFTER A PAUSE, MOVE THE BUCKET BACK TO PRESCORE AND LOWER THE LIFT
            } else if (toggle == 6 && getRuntime() >= IntakeTime + 1) {
                Bail();
                MoveBucket(BucketPreScorePos);
                MoveLift(LiftRestPos + liftAdjust, liftSpeed);
                down = true;
                toggle = -5; //reset toggle to intake stage
            }

            //TODO double check this code.  To me it looks like you are resetting the encoders after you
            //TODO move the lift up to 20ppr.  This would not be the correct way to do this.
            //TODO also, could the "down" toggle be removed so ANY TIME the Ltouch gets triggered
            //TODO the encoders get reset?
            if(Ltouch.isPressed() && down) {
                LiftR.setTargetPosition(20);
                LiftL.setTargetPosition(20);
                LiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LiftR.setPower(0.5);
                LiftL.setPower(0.5);
                LiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                down = false;
            }

            //TODO  could this simple code do the trick (ResetLiftEncoders() is below)
            if(Ltouch.isPressed()){
               //ResetLiftEncoders();
            }
/*
            //Manual Slide Movement
            if ( -gamepad2.left_stick_y >= .1) {
                // Manually raise the lift
                if (LiftR.getCurrentPosition() <= LiftDepositPos) {
                    MoveLift(LiftR.getCurrentPosition() + 20, .8);
                }

            } else if ( -gamepad2.left_stick_y <= -.1) {
                // Manually Lower the lift
                // if (LiftR.getCurrentPosition() >= LiftRestPos) {
                if(!Ltouch.isPressed()){
                    MoveLift(LiftR.getCurrentPosition() - 20, .8);
                } else if(Ltouch.isPressed()){
                    MoveLift(0, .8);
                }
            }*/

            //Code for manual lift correction
            if(gamepad2.triangle && button_State){
                liftAdjust = liftAdjust + 10;
                inches = liftAdjust * Lift_PPR_To_Inches;
                liftAdjustCount = liftAdjustCount + 1;
                button_State = false;
            } else if(gamepad2.cross && button_State){
                liftAdjust = liftAdjust - 10;
                inches = liftAdjust * Lift_PPR_To_Inches;
                liftAdjustCount = liftAdjustCount - 1;
                button_State = false;
            } else if(!gamepad2.triangle && !gamepad2.cross){
                button_State = true;
            }


            telemetry.addData("Is Basket High?", highToggle);
            telemetry.addData("Up/Down # of Button Presses", liftAdjustCount);
            telemetry.addData("Up/Down Adjust Value (in)", inches);
            telemetry.addData("X Pos", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y Pos", pos.getY(DistanceUnit.INCH));
            telemetry.addData("Heading Degrees", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Heading Radians", pos.getHeading(AngleUnit.RADIANS));
            telemetry.update();
        }//end of while loop

    }// end of run opmode loop

    private void Bail() {
        if (gamepad2.touchpad) {
            MoveLift(LiftRestPos, liftSpeed);
            MoveIntake(0);
            MoveWrist(RWristInPos, LWristInPos);
            MoveLinkage(RLinkageInPos, LLinkageInPos);

            // toggles value and starts a passive timer
            toggle = -5;
        }
    }

    private void MoveLift(int Position, double Speed) {

        if (Position > 0 && Position < 2400) {
            LiftR.setTargetPosition(Position);
            LiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftR.setPower(Speed);

            LiftL.setTargetPosition(Position);
            LiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftL.setPower(Speed);
        }
    }

    /**
     * Resets the encoders for the lift motors then moves the lift up to
     * "Rest" position.
     */
    private void ResetLiftEncoders(){
        LiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //MoveLift(LiftRestPos + liftAdjust, liftSpeed);
    }

    private void MoveLinkage(double Right, double Left) {
        Leftlinkage.setPosition(Left);
        Rightlinkage.setPosition(Right);
    }

    private void MoveWrist(double Right, double Left) {
        RWrist.setPosition(Right);
        LWrist.setPosition(Left);
    }

    private void MoveIntake(double Speed) {
        Lintake.setPower(Speed);
        Rintake.setPower(Speed);
    }

    private void MoveBucket(double Position) {
        Bucket.setPosition(Position);
    }
}