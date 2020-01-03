package com.team254.lib.util; 

public class LoggerMain {
    public static void main(String[] args) {
      LoggingSystem LS = LoggingSystem.getInstance();
       // creating a loggable object 
       ILoggable loggable = new TestLoggable();
       LS.Register(loggable, "Test.csv");
       // telling it to log on a regukar basis 
       for (int i = 0; i<10; i++) {
         LS.Log();
         try {
           Thread.sleep(1000);
         } catch (Exception e) {}
         
       }
    }
 }