/* real logger code
  TODO (EithneA-V)
    1) Create Logging System class
        1.1) Constructor function (base class pointer)
        1.2) Member var that is a list of obj to call when logging
            1.2.1) List of things it can call (and that can call it --> same that it can call)
            1.2.2) Can add/append/register whenever smthng new calls it
        1.3) Get data from list in 2.2
            1.3.1) Writes data to logging file (specify file name for each class)
        1.4) Create LS thread and update regularly
*/
package com.team254.lib.util;
import java.util.ArrayList;
import java.io.FileWriter;

public class LoggingSystem {
    // 1.1
    //  LoggableItems object
    ArrayList<ILoggable> loggable_items = new ArrayList<ILoggable>();
    //  creating a usingFileWriter object
    FileWriter fileWriter = new FileWriter("Test.csv");
    void LoggingSystem() {
    }
    void Register() {  //  start function that opens files
        try {
             // creating a loggable object 
            ILoggable loggable = new TestLoggable();
            //  adding Loggable to Loggable_items list
            loggable_items.add(loggable);
        } catch (Exception e) {}

    }
    void Log() {  //  function that gets called and told when to log by main
        try{
            for (int i=0; i < loggable_items.size(); i++) {
               ArrayList<Double> items = loggable_items.get(i).get_items();
               ArrayList<String> item_names = loggable_items.get(i).get_item_names();
               //  assertArrayEquals(items[i], item_names[i]);
               //  write to files
               for (int h=0; h < item_names.size(); h++) {
               fileWriter.write(item_names.get(h));
               fileWriter.write(",");
               }
               fileWriter.write("\n");
               for (int j=0; j < items.size(); j++) {
                fileWriter.write(items.get(j).toString());
                fileWriter.write(",");
               } 
               fileWriter.write("\n");
            }
        } catch (Exception e) {}  // making compiler happy by trying to catch stuff that could crash 
    }
    void Close() {  //  close file
        try {
            fileWriter.close();
        } catch (Exception e) {}

    }
    // 1.2
    public ArrayList<String> canCall;
    
}
