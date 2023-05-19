package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoTestRight extends AutoMaster{

   @Override
   public void runOpMode() throws InterruptedException{
      startSide = RIGHT;
      firstJunctionPos = Junction.MIDDLE;
      try {
         initHardware();
         longMoveNormal();
//         eject(Junction.MIDDLE,100);
      } catch (Exception e){
         throw new InterruptedException();
      }
   }
}
