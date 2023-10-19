package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Treasuremap {
    boolean y = false;
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
    HardwareMap Treasuremap = null;
    // creates runtime variable
    public ElapsedTime runtime = new ElapsedTime();

    public void init(HardwareMap hmap){
        //sets up names for configuration
        Treasuremap = hmap;

        Frontright = hmap.get(DcMotor.class,"RF");
        Backright = hmap.get(DcMotor.class,"RB");
        Backleft = hmap.get(DcMotor.class,"LB");
        Frontleft = hmap.get(DcMotor.class,"LF");
        LinAct = hmap.get(DcMotor.class,"Act");
        Claw = hmap.get(Servo.class,"C");
        Arm = hmap.get(DcMotor.class,"L");
        Wrist = hmap.get(Servo.class,"W");
        LinAngle = hmap.get(Servo.class,"LA");
        ArmAngle = hmap.get(DcMotor.class,"AA");
        Plane = hmap.get(Servo.class,"P");


        //Frontright.setDirection(DcMotor.Direction.REVERSE);
        //Backright.setDirection(DcMotor.Direction.REVERSE);


        // sets the lifts zeropowerbehavior to brake
        Frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
/*
        //set claw open posibly
        Claw.setPosition(.5);

        // set wrist angle right ... maybe
        Wrist.setPosition(.5);

        // set plane lanch angle not launch it yet.... in theroy
        Plane.setPosition(.5);

        // linear actuar start angle .... hopefully
        LinAngle.setPosition(.5);
*/
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LinAct.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinAct.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinAct.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
    // function for driving forward
    //runs motors forward at 60% power
    public void Forward(double seconds) {
        runtime.reset();
        while (runtime.milliseconds() < (seconds * 1000)) {
            Frontleft.setPower(0.6);
            Frontright.setPower(0.6);
            Backleft.setPower(0.6);
            Backright.setPower(0.6);
        }
    }
    //function for driving backward
    //runs motors backward at 60% power
    public void Backward(double seconds){
        runtime.reset();
        while (runtime.milliseconds()<(seconds*1000)){
            Frontleft.setPower(-0.6);
            Frontright.setPower(-0.6);
            Backleft.setPower(-0.6);
            Backright.setPower(-0.6);
        }
    }

    // function for turning left
    //runs left motors backward at 60% power
    //runs right motors forward at 60% power
    public void Left(double seconds){
        runtime.reset();
        while (runtime.milliseconds()<(seconds*1000)){
            Frontleft.setPower(-0.6);
            Frontright.setPower(0.6);
            Backleft.setPower(-0.6);
            Backright.setPower(0.6);
        }
    }
    // function for turning right
    //runs right motors backward at 60% power
    //runs left motors forward at 60% power
    public void Right(double seconds){
        runtime.reset();

        while (runtime.milliseconds() < (seconds * 1000)){
            Frontleft.setPower(0.6);
            Frontright.setPower(-0.6);
            Backleft.setPower(0.6);
            Backright.setPower(-0.6);

        }
    }
    // function for strafing right
    //runs frontleft motor forward at 60% power
    //runs frontright motor backward at 60% power
    //runs backleft motor backward at 60% power
    //runs backright motor forward at 60% power
    public void RightStrafe(double seconds){
        runtime.reset();

        while (runtime.milliseconds() < (seconds * 1000)){
            Frontleft.setPower(0.6);
            Frontright.setPower(-0.6);
            Backleft.setPower(-0.6);
            Backright.setPower(0.6);

        }
    }
    // function for strafing left
    //runs frontleft motor backward at 60% power
    //runs frontright motor forward at 60% power
    //runs backleft motor forward at 60% power
    //runs backright motor backward at 60% power
    public void LeftStrafe(double seconds){
        runtime.reset();
        while(runtime.milliseconds()<(seconds*1000)){
            Frontleft.setPower(-0.6);
            Frontright.setPower(0.6);
            Backleft.setPower(0.6);
            Backright.setPower(-0.6);
        }
    }


    // function for turning off motors
    public void Off(){
        Frontright.setPower(0);
        Frontleft.setPower(0);
        Backleft.setPower(0);
        Backright.setPower(0);
    }

    public void Aoff(){
        Arm.setPower(0);
    }

    //claw grab
    public void C(double position) {Claw.setPosition(position);}
    //claw wrist
    public void W(double position) {Wrist.setPosition(position);}
    // plane launch
    public void P(double position) {Plane.setPosition(position);}

    public void LA(double position) {LinAngle.setPosition(position);}

    //function for moving arm 1 down
    //runs arm 1 motor down at 100% power
    public void AD (double seconds){
        double time = (seconds * 1000) + runtime.milliseconds();
        while (time > runtime.milliseconds()) {
            Arm.setPower(1);
        }

    }
    //function for moving arm 1 up
    // runs arm 1 motor up at 100% power
    public void AU (double seconds){
        double time = (seconds * 1000) + runtime.milliseconds();
        while (time > runtime.milliseconds()) {
            Arm.setPower(-1);
        }
    }

    //function for moving arm 1 down
    //runs arm 1 motor down at 100% power
    public void ArmAU (double seconds){
        double time = (seconds * 1000) + runtime.milliseconds();
        while (time > runtime.milliseconds()) {
            Arm.setPower(1);
        }

    }
    //function for moving arm 1 up
    // runs arm 1 motor up at 100% power
    public void ArmAD (double seconds){
        double time = (seconds * 1000) + runtime.milliseconds();
        while (time > runtime.milliseconds()) {
            Arm.setPower(-1);
        }
    }


    //function for moving arm 1 down
    //runs arm 1 motor down at 100% power
    public void LINU (double seconds){
        double time = (seconds * 1000) + runtime.milliseconds();
        while (time > runtime.milliseconds()) {
            LinAct.setPower(1);
        }

    }
    //function for moving arm 1 up
    //runs arm 1 motor up at 100% power
    public void LIND (double seconds){
        double time = (seconds * 1000) + runtime.milliseconds();
        while (time > runtime.milliseconds()) {
            LinAct.setPower(-1);
        }
    }

}




