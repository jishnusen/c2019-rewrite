//  main logging function 
//  import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import java.util.ArrayList;
import java.io.FileWriter; 
public class LoggerMain {
    public static void main(String[] args) {
        // creating a loggable object 
        ILoggable loggable = new TestLoggable();
        //  creating a usingFileWriter object
        try{
        FileWriter fileWriter = new FileWriter("Test.csv");
        //  LoggableItems object
        ArrayList<ILoggable> loggable_items = new ArrayList<ILoggable>();
        //  adding Loggable to Loggable_items list
        loggable_items.add(loggable);
        //  for loop will loop over anything that implements the logger
        for (int i=0; i < loggable_items.size(); i++) {
           ArrayList<Double> items = loggable_items.get(i).get_items();
           ArrayList<String> item_names = loggable_items.get(i).get_item_names();
           //  assertArrayEquals(items[i], item_names[i]);
           //  write to files
           // fileWriter.write(items);
           //  fileWrter.write(item_names);
           fileWriter.write(items.get(0).toString());
           fileWriter.write(item_names.get(0));
        }
        fileWriter.close(); // makes sure its actually written
    } catch (Exception e) {}  // making compiler happy by trying to catch stuff that could crash 
    }
 }