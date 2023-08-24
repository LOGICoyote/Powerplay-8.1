/* Copyright Lydia & M1
Take it, I don't care
Programming is just copying
 */


package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp
public class FancyLEDTest extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //Hardware
    private RevBlinkinLedDriver leds;
    private RevColorSensorV3 color;

    //Variables

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        leds = hardwareMap.get(RevBlinkinLedDriver.class, "LEDS");
        color = hardwareMap.get(RevColorSensorV3.class, "color");
    }


    @Override
    public void start() {
        runtime.reset();

    }

    @Override
    public void loop() {
        //if (gamepad1.y){
           // leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);}
        //if (gamepad1.b){
          //  leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);}
       // if (gamepad1.a){
            //leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);}
        telemetry.addData("dist",color.getDistance(DistanceUnit.MM));
        telemetry.addData("red",color.red());
        if (color.getDistance(DistanceUnit.MM)> 60){
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);}
        if (color.getDistance(DistanceUnit.MM)>20 && (color.getDistance(DistanceUnit.MM)<=60)){
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);}
        if (color.getDistance(DistanceUnit.MM)<=20){
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);}

    }


    @Override
    public void stop() {
    }
}