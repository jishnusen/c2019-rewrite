/*what is loggable? --> implimented by subsystems 
 has data to be logged
 implements ILogger.java*/
 package com.team254.lib.util;
 import com.team254.lib.util.*;
 import java.util.ArrayList;
 
 public interface ILoggable {
     ArrayList<Double> get_items();
     ArrayList<String> get_item_names();
 }