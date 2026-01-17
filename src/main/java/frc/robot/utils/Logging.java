package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.elasticFiles.CoreElastic;
import frc.robot.utils.elasticFiles.CoreElastic.Notification;

public class Logging {

    @SuppressWarnings("resource")
    public static void stickyWarning(String title, String description)
    {
        if(description == null) description = title;

        warningMsg(title, description);
        new Alert(description, AlertType.kWarning).set(true);
    }

    @SuppressWarnings("resource")
    public static void stickyError(String title, String description)
    {
        if(description == null) description = title;

        errorMsg(title, description);
        new Alert(description, AlertType.kError).set(true);
    }
    
    public static void infoMsg(String title, String description)
    {
        if(description == null) description = title;

        CoreElastic.Notification notification = new CoreElastic.Notification(CoreElastic.NotificationLevel.INFO, title,description);
        CoreElastic.sendNotification(notification);

        System.out.println("[INFO] " + title + ": " + description);
    }
    
    public static void warningMsg(String title, String description)
    {
        if(description == null) description = title;

        CoreElastic.Notification notification = new CoreElastic.Notification(CoreElastic.NotificationLevel.WARNING, title,description);
        CoreElastic.sendNotification(notification);

        DriverStation.reportWarning(description, false);

        System.out.println("[WARNING] " + title + ": " + description);
    }

    public static void errorMsg(String title, String description)
    {
        if(description == null) description = title;
        
        CoreElastic.Notification notification = new CoreElastic.Notification(CoreElastic.NotificationLevel.ERROR, title,description);
        CoreElastic.sendNotification(notification);

        DriverStation.reportError(description, false);

        System.err.println("[ERROR] " + title + ": " + description);
    }
}
