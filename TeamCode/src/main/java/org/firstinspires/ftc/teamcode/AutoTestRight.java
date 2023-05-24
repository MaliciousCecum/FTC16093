package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoTestRight extends AutoMaster{

   @Override
   public void runOpMode() throws InterruptedException{
      startSide = RIGHT;
      firstJunctionPos = Junction.SIDE_HIGH;
      initHardware();
      try {
         longMoveNormal();
         eject(firstJunctionPos, 200);
         for (cone_index = 4; cone_index >= 0; cone_index--) {
            try {
               intake(cone_index, firstJunctionPos);
            } catch (Configurations.TimeoutException e) {
               intakeSave();
            }
            moveToEject();
            eject(firstJunctionPos, 300);
         }
         breakPoint();
         park();
      } catch (Configurations.GlobalTimeoutException e) {
         park();
      } catch (Exception e) {
         throw new InterruptedException();
      } finally {
         savePosition();
      }
   }
}
