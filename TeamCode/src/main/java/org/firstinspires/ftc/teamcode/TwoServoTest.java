package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;


@TeleOp
@Config
public class TwoServoTest extends LinearOpMode {

   public static double servoPos0 = 0.5;
   public static double servoPos1 = 0.5;
   public static double servoPosDiff = 0;
   public static String servoName0 = "s0";
   public static String servoName1 = "s1";

   @Override
   public void runOpMode() {
      List<LynxModule>  allHubs = hardwareMap.getAll(LynxModule.class);
      for (LynxModule module : allHubs) {
         module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
      }
      Servo servo0 = hardwareMap.get(Servo.class, servoName0);
      Servo servo1 = hardwareMap.get(Servo.class, servoName1);

      waitForStart();
      while (opModeIsActive()) {
         servo0.setPosition(servoPos0 - servoPosDiff);
         servo1.setPosition(servoPos1 + servoPosDiff);
         for (LynxModule module : allHubs) {
            module.clearBulkCache();
         }
         idle();
      }
   }
}
