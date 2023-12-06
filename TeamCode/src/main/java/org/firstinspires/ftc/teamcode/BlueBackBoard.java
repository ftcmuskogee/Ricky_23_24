/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

/*
 * This sample demonstrates how to run analysis during INIT
 * and then snapshot that value for later use when the START
 * command is issued. The pipeline is re-used from SkystoneDeterminationExample
 */
@Autonomous(name = "BlueBackBoard", group = "robot")
public class BlueBackBoard extends LinearOpMode {
    OpenCvWebcam webcam;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    int ID_TAG_OF_INTEREST_L = 1;
    int ID_TAG_OF_INTEREST_C = 2;
    int ID_TAG_OF_INTEREST_R = 3;
    AprilTagDetection tagOfInterest = null;
    OpenCvBlue.BlueDeterminationPipeline pipeline;
    OpenCvBlue.BlueDeterminationPipeline.SkystonePosition snapshotAnalysis = OpenCvBlue.BlueDeterminationPipeline.SkystonePosition.CENTER; // default

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new OpenCvBlue.BlueDeterminationPipeline();
        webcam.setPipeline(pipeline);
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_OF_INTEREST_L) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addLine("Left tag is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_OF_INTEREST_C) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addLine("Center tag is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_OF_INTEREST_R) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addLine("Right tag is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }

                else {
                    telemetry.addLine("Don't see tag of interest :(");
                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        Treasuremap robot = new Treasuremap();
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-35, 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        TrajectorySequence M = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-35, 37), SampleMecanumDrive.getVelocityConstraint(58, Math.toRadians(360), 13.5),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .build();
        TrajectorySequence M2 = drive.trajectorySequenceBuilder(M.end())
                .lineToConstantHeading(new Vector2d(-35, 57), SampleMecanumDrive.getVelocityConstraint(58, Math.toRadians(360), 13.5),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .lineToConstantHeading(new Vector2d(-10, 57), SampleMecanumDrive.getVelocityConstraint(58, Math.toRadians(360), 13.5),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .build();
        TrajectorySequence L = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-35, 40), SampleMecanumDrive.getVelocityConstraint(58, Math.toRadians(360), 13.5),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .turn(Math.toRadians(50))
                .build();
        TrajectorySequence L2 = drive.trajectorySequenceBuilder(L.end())
                .back(1)
                .turn(Math.toRadians(-50))
                .lineToConstantHeading(new Vector2d(-35, 57), SampleMecanumDrive.getVelocityConstraint(58, Math.toRadians(360), 13.5),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .lineToConstantHeading(new Vector2d(-10, 57), SampleMecanumDrive.getVelocityConstraint(58, Math.toRadians(360), 13.5),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .build();
        TrajectorySequence R = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-35, 40), SampleMecanumDrive.getVelocityConstraint(58, Math.toRadians(360), 13.5),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .turn(Math.toRadians(-52))
                .build();
        TrajectorySequence R2 = drive.trajectorySequenceBuilder(R.end())
                .back(1)
                .turn(Math.toRadians(52))
                .lineToConstantHeading(new Vector2d(-35, 56), SampleMecanumDrive.getVelocityConstraint(58, Math.toRadians(360), 13.5),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .lineToConstantHeading(new Vector2d(-9, 56), SampleMecanumDrive.getVelocityConstraint(58, Math.toRadians(360), 13.5),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .build();


        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */
        snapshotAnalysis = pipeline.getAnalysis();

        /*
         * Show that snapshot on the telemetry
         */
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        switch (snapshotAnalysis) {
            case LEFT: {

                telemetry.addLine("left");
                robot.C(0);
                sleep(500);
                robot.W(.65);
                sleep(500);
                drive.followTrajectorySequence(L);
                sleep(500);
                robot.C(0.2);
                sleep(500);
                drive.followTrajectorySequence(L2);
                if (tagOfInterest.pose.x <= 20) {
                    robot.Forward(1);
                }
                else if (tagOfInterest.pose.x >= 20 && tagOfInterest.pose.x <= 50) {
                    // do something else
                }
                else if (tagOfInterest.pose.x >= 50) {
                    // do something else
                }
                break;
            }

            case RIGHT: {
                telemetry.addLine("right");
                robot.C(0);
                sleep(500);
                robot.W(.65);
                sleep(500);
                drive.followTrajectorySequence(R);
                sleep(500);
                robot.C(0.2);
                drive.followTrajectorySequence(R2);
                break;
            }

            case CENTER: {
                telemetry.addLine("mid");
                robot.C(0);
                sleep(500);
                robot.W(.65);
                sleep(500);
                drive.followTrajectorySequence(M);
                sleep(500);
                robot.C(0.2);
                drive.followTrajectorySequence(M2);
                break;
            }
        }
    }
    void tagToTelemetry (AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
}
