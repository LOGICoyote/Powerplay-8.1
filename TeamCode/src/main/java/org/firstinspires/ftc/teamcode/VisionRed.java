


package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous

public class VisionRed extends LinearOpMode {

    OpenCvCamera WebCam;
    Thing myThing;
    static int pos = 0;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor arm = null;
    private DcMotor intake;
    private Servo mailbox;

    //private Rev2mDistanceSensor dist;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
//        OR = hardwareMap.get(Servo.class, "OR");
//        OL = hardwareMap.get(Servo.class, "OL");
//        OS = hardwareMap.get(Servo.class, "OS");
        //dist = hardwareMap.get(Rev2mDistanceSensor.class, "dist");
        //OL.setPosition(0);
        //OR.setPosition(1);
        //OS.setPosition(0.8);


        mailbox = hardwareMap.get(Servo.class, "mailbox");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        intake = hardwareMap.get(DcMotor.class, "intake");
        arm = hardwareMap.get(DcMotor.class, "arm");


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        myThing = new Thing();
        WebCam.setPipeline(myThing);
        WebCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

        @Override
            public void onOpened() {
                WebCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }
        @Override
            public void onError(int errorCode) {
                }
            });
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(8, -64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(
                        //(-10,-47)
                        new Vector2d(-10, -44),Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(2, () -> {
                    armuphigh();
                })
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), true)
                .splineTo(new Vector2d(8, -65),Math.toRadians(0))
                .build();
        Trajectory traj2fix = drive.trajectoryBuilder(traj2.end(), true)
                .strafeLeft(2)
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2fix.end())
                .back(15)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), true)
                //55, -64
                //.splineTo(new Vector2d(55,-64),Math.toRadians(0))
                .back(25)
                .build();
        Trajectory traj4fix = drive.trajectoryBuilder(traj4.end(), true)
                //55, -64
                .strafeLeft(3)
                .build();
        Trajectory traj4park = drive.trajectoryBuilder(traj3.end(), true)
                //55, -64
                .forward(5)
                .build();
        Trajectory traj5= drive.trajectoryBuilder(traj4fix.end())
                //15
                .forward(10)
                .build();
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end())
                .splineTo(
                        new Vector2d(-10, -44),Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(2, () -> {
                    armuphigh();
                })
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end(), true)
                .splineTo(new Vector2d(8, -65),Math.toRadians(0))
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .strafeLeft(5)
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .back(30)
                .build();


        TrajectorySequence PosB1 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(
                        new Vector2d(-10, -54),Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(2, () -> {
                    armupmid();
                })
                .build();

        Trajectory trajB2 = drive.trajectoryBuilder(PosB1.end(), true)
                //-65
                .splineTo(new Vector2d(8, -65),Math.toRadians(0))
                .build();
        Trajectory trajB2fix = drive.trajectoryBuilder(trajB2.end(), true)
                .strafeLeft(2)
                .build();
        TrajectorySequence trajB3 = drive.trajectorySequenceBuilder(trajB2fix.end())
                //3
                .back(15)
                .build();
        Trajectory B4 = drive.trajectoryBuilder(trajB3.end(), true)
                //55, -64
                .splineTo(new Vector2d(55,-64),Math.toRadians(0))
                .build();
        TrajectorySequence trajA1 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(
                        new Vector2d(-10, -58),Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(2, () -> {
                    armuplow();
                })
                .build();
        Trajectory trajA2 = drive.trajectoryBuilder(trajA1.end(), true)
                .splineTo(new Vector2d(8, -65),Math.toRadians(0))
                .build();
        Trajectory trajA2fix = drive.trajectoryBuilder(trajA2.end(), true)
                .strafeLeft(2)
                .build();
        TrajectorySequence trajA3 = drive.trajectorySequenceBuilder(trajA2fix.end())
                //3
                .back(15)
                .build();
        Trajectory trajA4 = drive.trajectoryBuilder(trajA3.end(), true)
                //55, -64
                .splineTo(new Vector2d(55,-64),Math.toRadians(0))
                .build();


        //WebCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
            waitForStart();
            holding();
            //while (opModeIsActive()) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addData("Analysis", myThing.getAnalysis());
                telemetry.addData("Position", myThing.position);
               // sleep(2000);

             if (myThing.position == Thing.CapPosition.C){
                 drive.followTrajectorySequence(traj1);
                 score();
                 armdown();
                 drive.followTrajectory(traj2);
                 drive.followTrajectory(traj2fix);
                 drive.followTrajectorySequence(traj3);
                 intake.setPower(1);
                 drive.followTrajectory(traj4);
                 drive.followTrajectory(traj4fix);
                 holding();
                 intake.setPower(-0.6);
                 drive.followTrajectory(traj5);
                 intake.setPower(0);
                 drive.followTrajectorySequence(traj6);
                 score();
                 armdown();
                 drive.followTrajectory(traj7);
                 drive.followTrajectory(traj8);
                 drive.followTrajectory(traj9);
                 sleep(999999);
             }
             else if (myThing.position == Thing.CapPosition.B){
                drive.followTrajectorySequence(PosB1);
                score();
                armdown();
                drive.followTrajectory(trajB2);
                 drive.followTrajectory(trajB2fix);
                drive.followTrajectorySequence(trajB3);
                 intake.setPower(1);
                 drive.followTrajectory(traj4);
                 holding();
                 drive.followTrajectory(traj4fix);
                 intake.setPower(-0.6);
                 drive.followTrajectory(traj5);
                 intake.setPower(0);
                 drive.followTrajectorySequence(traj6);
                 score();
                 armdown();
                 drive.followTrajectory(traj7);
                 drive.followTrajectory(traj8);
                 drive.followTrajectory(traj9);
                 sleep(999999);
             }
             else if (myThing.position == Thing.CapPosition.A){
                 drive.followTrajectorySequence(trajA1);
                 score();
                 armdown();
                 drive.followTrajectory(trajA2);
                 drive.followTrajectory(trajA2fix);
                 drive.followTrajectorySequence(trajA3);
                 intake.setPower(1);
                 drive.followTrajectory(traj4);
                 holding();
                 drive.followTrajectory(traj4fix);
                 intake.setPower(-0.6);
                 drive.followTrajectory(traj5);
                 intake.setPower(0);
                 drive.followTrajectorySequence(traj6);
                 score();
                 armdown();
                 drive.followTrajectory(traj7);
                 drive.followTrajectory(traj8);
                 drive.followTrajectory(traj9);
                 sleep(999999);
            }
            }
       // }
    private void armuphigh(){
        arm.setTargetPosition(425);
        if (arm.getCurrentPosition() >= 425) {
            arm.setPower(0);
        }
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);
    }
        private void armupmid(){//550 mid
            arm.setTargetPosition(550);
            if (arm.getCurrentPosition() >= 550) {
                arm.setPower(0);
            }
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);}
    private void armuplow(){
        arm.setTargetPosition(650);
        if (arm.getCurrentPosition() >= 650) {
            arm.setPower(0);
        }
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);}
    private void dump(){
        mailbox.setPosition(0.4);
    }
    private void accepting(){
        mailbox.setPosition(0);
    }
    private void holding(){
        mailbox.setPosition(0.1);
    }
    private void armdown(){
        arm.setTargetPosition(0);
        if (arm.getCurrentPosition() <= 0) {
            arm.setPower(0);
        }
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);
    }
    private void score() {
        dump();
        sleep(1000);
        accepting();
    }

    public static class Thing extends OpenCvPipeline {
        public enum CapPosition {
            A,
            B,
            C
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Point TopLeftPoint = new Point(10, 80);
        static final Point TopLeftPoint2 = new Point(200, 80);
       // static final Point TopLeftPoint3 = new Point(100, 115);
        static final int Region_width = 15;
        static final int Region_height = 60;
        Point region2_pointA = new Point(TopLeftPoint2.x, TopLeftPoint2.y);
        Point region2_pointB = new Point(TopLeftPoint2.x + Region_width, TopLeftPoint2.y + Region_height);
        Point region1_pointA = new Point(TopLeftPoint.x, TopLeftPoint.y);
        Point region1_pointB = new Point(TopLeftPoint.x + Region_width, TopLeftPoint.y + Region_height);
        Mat region1_Cr;
        Mat region2_Cr;
        Mat YCrCb = new Mat();
        Mat Cr = new Mat();
        int avg1;
        int avg2;
        int iscap = 145;
        private volatile CapPosition position = CapPosition.A;
        void inputToCb(Mat input){
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb,Cr,1);
        }
        @Override
        public void init (Mat firstFrame){
           inputToCb(firstFrame);
            region1_Cr= Cr.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cr= Cr.submat(new Rect(region2_pointA, region2_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            avg1 = (int)Core.mean(region1_Cr).val[0];
            avg2 = (int)Core.mean(region2_Cr).val[0];


            Imgproc.rectangle(input, region1_pointA, region1_pointB, BLUE, 2);
            Imgproc.rectangle(input, region2_pointA, region2_pointB, BLUE, 2);


            position = CapPosition.A;
            if (avg1 > iscap){
                position = CapPosition.B;
            }
            else if (avg2 > iscap){
                position = CapPosition.A;
            }
            else{
            position = CapPosition.C;
            }
            Imgproc.rectangle(input,region1_pointA,region1_pointB, GREEN, 1);
            Imgproc.rectangle(input,region2_pointA,region2_pointB, GREEN, 1);
            return input;
        }
        public int getAnalysis(){
            return avg1;
        }

    }

}


