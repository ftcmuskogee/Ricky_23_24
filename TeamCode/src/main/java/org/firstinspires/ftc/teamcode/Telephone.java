package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@TeleOp(name="Telephone")

public class Telephone extends LinearOpMode {
    // names motors and sets motors to type null
    public DcMotor Frontright = null;
    public DcMotor Backright = null;
    public DcMotor Backleft = null;
    public DcMotor Frontleft = null;
    public DcMotor LinAct = null;
    public Servo Claw = null;
    // wrist
    public Servo Wrist = null;
    // linear actuator angle change
    public Servo LinAngle = null;
    // arm angle
    public DcMotor ArmAngle = null;
    public DcMotor Arm = null;
    //plane launch
    public Servo Plane = null;
    // sets hardware map to null and names it

    //sets booleans to false
    private final boolean isPressed = false;
    public boolean buttonPressed = false;
    //sets variables to 0
    public double lastTick = 0;
    public double lastTime = 0;
    public double currentTick = 0;
    public double currentTime = 0;
    public double targetShooterRPM = 0;


    //calls hardware map
    Treasuremap robot = new Treasuremap();

    @Override
    public void runOpMode() {
        double speed;

        //sets up names for configuration
        Frontright = hardwareMap.get(DcMotor.class, "RF");
        Backright = hardwareMap.get(DcMotor.class, "RB");
        Backleft = hardwareMap.get(DcMotor.class, "LB");
        Frontleft = hardwareMap.get(DcMotor.class, "LF");
        LinAct = hardwareMap.get(DcMotor.class, "Act");
        Claw = hardwareMap.get(Servo.class, "C");
        Arm = hardwareMap.get(DcMotor.class, "L");
        Wrist = hardwareMap.get(Servo.class, "W");
        LinAngle = hardwareMap.get(Servo.class, "LA");
        ArmAngle = hardwareMap.get(DcMotor.class, "AA");
        Plane = hardwareMap.get(Servo.class, "P");

        // sets the right 2 motors to reverse
        /*Frontright.setDirection(DcMotor.Direction.REVERSE);
        Backright.setDirection(DcMotor.Direction.REVERSE);*/

        Frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LinAct.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinAct.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinAct.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
/*
        //set claw open posibly
        Claw.setPosition(.5);
        // set wrist angle right ... maybe
        Wrist.setPosition(.5);
        // set plane lanch angle not launch it yet.... in theroy
        Plane.setPosition(.5);
        // linear actuar start angle .... hopefully
        LinAngle.setPosition(.5);*/

        //calls from samplemecanumdrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //sets motors to run without encoders
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //initializes hardware map
        //robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            //sets all 4 base motors to the left and right joysticks on gamepad 1
            //uses the variables from SampleMecanumDrive to adjust motors
            //left stick in the y direction is for going forward and backward at 80% power
            //left stick in the x direction is for strafing left and right at 80% power
            //right stick in the x direction is for turning left and right at 80% power
            //was x: y
            //was y: x
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 1,
                            -gamepad1.left_stick_x * 1,
                            -gamepad1.right_stick_x * 0.8
                    )
            );

            drive.update();

            /** figure out what dis specifically do **/

            // Makes variables Power1 and Power2 to their respective joystick
            double Power1 = gamepad2.right_stick_y;
            double Power2 = gamepad2.left_stick_y;
            speed = -.2;
            // sets the power for the lifts
            Arm.setPower(Power1 * speed);
            ArmAngle.setPower(Power2 * speed);

            //down
            if (gamepad1.right_bumper) {
                LinAct.setPower(-.2);
            }
            //up
            else if (gamepad1.left_bumper) {
                LinAct.setPower(.2);
            } else {
                LinAct.setPower(0);
            }
//turn servo not positiion
            if (gamepad1.a) {
                LinAngle.setPosition(-.05);
            }
            //up idk
            if (gamepad2.right_bumper){
                Wrist.setPosition(.65);
            }
            // down idk
            if (gamepad2.left_bumper){
                Wrist.setPosition(.8);
            }
            //close
            if (gamepad2.left_trigger >0.1) {
                Claw.setPosition(.1);
            }

            if (gamepad2.right_trigger > .1){
                Claw.setPosition(0);
            }
/*
            if (gamepad2.left_trigger > 0.1) {
                Wrist.setPosition(.8);
            }
            else if (gamepad2.right_trigger > 0.1) {
                Wrist.setPosition(.65);
            }
*/
            if (gamepad2.a) {
                Plane.setPosition(.2);
            }
/*
            if (gamepad2.right_bumper) {
                Claw.setPosition(.1);
            }

            //Closes claws when the left bumper on gamepad 2 is pressed
            else if (gamepad2.left_bumper) {
                Claw.setPosition(0);
            }*/

            //adds data to the driver hub that tells you the coordinates of where the robot is on the field
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("a", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}


