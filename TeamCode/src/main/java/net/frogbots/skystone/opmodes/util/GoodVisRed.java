


package net.frogbots.skystone.opmodes.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

public class GoodVisRed extends LinearOpMode {

    OpenCvCamera WebCam;
    Thing myThing;
    static int pos = 0;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor lift = null;
    private Servo claw;
    private Servo extend;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        leftFront = hardwareMap.get(DcMotor.class, "motor00");
        rightFront = hardwareMap.get(DcMotor.class, "motor01");
        leftBack = hardwareMap.get(DcMotor.class, "motor02");
        rightBack = hardwareMap.get(DcMotor.class, "motor03");
        claw=hardwareMap.get(Servo.class,"claw");

        extend = hardwareMap.get(Servo.class,"extendy");
        lift =  hardwareMap.get(DcMotor.class, "motor 3");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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

        Pose2d startPose = new Pose2d(70, 40, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(3)
                .forward(35)
                .back(5)
                .turn(Math.toRadians(-182))
                .strafeLeft(25)
                .forward(3)
                .strafeLeft(8)
                 .forward(1)
                //.forward(2)
//                .addDisplacementMarker( () -> {
//
//                })
                .build();
        TrajectorySequence traj2a = drive.trajectorySequenceBuilder(traj1.end())
                .strafeRight(9)
                .build();
        TrajectorySequence traj2b = drive.trajectorySequenceBuilder(traj1.end())
                .strafeRight(36)
                .build();
        TrajectorySequence traj2c = drive.trajectorySequenceBuilder(traj1.end())
                .strafeRight(58)
                .back(10)
                .build();

        waitForStart();
        sleep(3000);
        telemetry.addData("Analysis", myThing.getAnalysis());
        telemetry.addData("position", myThing.position);
        clawopen();
           if (myThing.position == Thing.CapPosition.C){
               drive.followTrajectorySequence(traj1);
               lift(265);
               sleep(300);
               lift(780);
               sleep(300);
               extend.setPosition(.65);
               sleep(2000);
               clawclosed();
               sleep(1500);
               extend.setPosition(.35);
               sleep(500);
               lift(0);
               drive.followTrajectorySequence(traj2a);
               sleep(999999);            }
             else if (myThing.position == Thing.CapPosition.B){
               drive.followTrajectorySequence(traj1);
               lift(265);
               sleep(300);
               lift(780);
               sleep(300);
               extend.setPosition(.65);
               sleep(2000);
               clawclosed();
               sleep(1500);
               extend.setPosition(.35);
               sleep(500);
               lift(0);
               drive.followTrajectorySequence(traj2b);
               sleep(999999);
             }
            else if (myThing.position == Thing.CapPosition.A){
               drive.followTrajectorySequence(traj1);
               lift(265);
               sleep(300);
               lift(780);
               sleep(300);
               extend.setPosition(.65);
               sleep(2000);
               clawclosed();
               sleep(1500);
               extend.setPosition(.35);
               sleep(500);
               lift(0);
                    drive.followTrajectorySequence(traj2c);
                     sleep(999999);
           }
    }

       private void lift(int liftpos) {
           double lift_power = (liftpos - lift.getCurrentPosition()) * 0.002;
           //.002
           lift.setTargetPosition(liftpos);
           if (lift.getCurrentPosition() >= liftpos) {
               lift.setPower(-1);
           }
           lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           lift.setPower(lift_power);
           telemetry.addData("lift power", lift_power);
       }
    private void clawopen(){
        claw.setPosition(.4);
    }
    private void clawclosed(){
        claw.setPosition(.55);
    }

    public static class Thing extends OpenCvPipeline {
        public enum CapPosition {
            A,
            B,
            C
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Point TopLeftPoint = new Point(75, 45);
        static final int Region_width = 50;
        static final int Region_height = 70;
        Point region1_pointA = new Point(TopLeftPoint.x, TopLeftPoint.y);
        Point region1_pointB = new Point(TopLeftPoint.x + Region_width, TopLeftPoint.y + Region_height);
        Mat region1_Cr;
        Mat region2_Cr;
        Mat YCrCb = new Mat();
        Mat Cr = new Mat();
        int avg1;

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
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            avg1 = (int)Core.mean(region1_Cr).val[0];
            Imgproc.rectangle(input, region1_pointA, region1_pointB, BLUE, 2);


            position = CapPosition.A;
            if (avg1 < 130){
                position = CapPosition.A;
            }
            else if ((avg1<135)&&(avg1>=130)){
                position = CapPosition.B;
            }
            else if (avg1>=135){
            position = CapPosition.C;
            }
            Imgproc.rectangle(input,region1_pointA,region1_pointB, GREEN, 1);
            return input;
        }
        public int getAnalysis(){
            return avg1;
        }
    }


}