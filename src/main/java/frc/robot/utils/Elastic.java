package frc.robot.utils;

import frc.robot.utils.elasticFiles.CoreElastic;

public class Elastic {
    
    public static void infoMsg(String title, String description)
    {
        CoreElastic.Notification notification = new CoreElastic.Notification(CoreElastic.NotificationLevel.INFO, title,description);
        CoreElastic.sendNotification(notification);
    }
    
    public static void warningMsg(String title, String description)
    {
        CoreElastic.Notification notification = new CoreElastic.Notification(CoreElastic.NotificationLevel.WARNING, title,description);
        CoreElastic.sendNotification(notification);
    }

    public static void errorMsg(String title, String description)
    {
        CoreElastic.Notification notification = new CoreElastic.Notification(CoreElastic.NotificationLevel.ERROR, title,description);
        CoreElastic.sendNotification(notification);
    }
}
