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
import java.util.List;

public class Logger {
    // 1.1
    void LoggingClass() {
    }
    // 1.2
    public List[] canCall;
    
}
