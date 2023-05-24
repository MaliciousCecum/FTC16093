package org.firstinspires.ftc.teamcode;

import java.util.function.BooleanSupplier;

public class LJBBoolean extends XCYBoolean{
   int timeMM;
   long lastFalseTime;
   public LJBBoolean(BooleanSupplier condition, int time) {
      super(condition);
      timeMM = time;
   }

   public void read(){
      super.read();
      if (!super.get()) lastFalseTime=System.currentTimeMillis();
   }

   public boolean get(){
      return System.currentTimeMillis()>lastFalseTime+timeMM;
   }
}
