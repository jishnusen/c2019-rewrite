/*what is loggable? --> implimented by subsystems 
 has data to be logged
 implements ILogger.java*/
public interface ILoggable{
    ArrayList<float> get_items();
    ArrayList<String> get_item_names();
}