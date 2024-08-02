#ifndef INC_NOTIFICATION_H_
#define INC_NOTIFICATION_H_

#include "gpio.h"
#include "timer.h"

enum NotificationLevel {
    kNotificationBatteryCoverRemoved = 0,
    kNotificationServiceModeEntered,
    kNotificationServiceModeComplete,
    kNotificationDataAcquisitionStarted,
    kNotificationDataAcquisitionOngoing,
    kNotificationBatteryNeedsCharging,
    kNotificationDoorOpenedDuringDataAcquisition,
    kNotificationDoorOpened,
    kNotificationObjDetected,
    kNotificationBatteryCoverRemovedDuringDataAcquisition,
    kNotificationMax
};

class Notification : public TimerMonitor {
public:
    Notification();
    virtual ~Notification();

    bool SetNotification(NotificationLevel notification);
    bool ClearNotification(NotificationLevel notification);

    void TimerCallback(int8_t id);
    void Notify();

private:
    bool notifications_[kNotificationsMax];
    NotificationLevel level_;
    uint32_t counter_;

    bool ValidateLevel(NotificationLevel level);
    void ClearUI();
    void ClearTimer();
    bool NotifyBatteryCoverRemoved();
    bool NotifyServiceModeEntered();
    bool NotifyServiceModeComplete();
    bool NotifyDataAcquisitionStarted();
    bool NotifyDataAcquisitionOngoing();
    bool NotifyBatteryNeedsCharging();
    bool NotifyDoorOpenedDuringDataAcquisition();
    bool NotifyDoorOpened();
    bool NotifyObjDetected();
    bool NotifyBatteryCoverRemovedDuringDataAcquisition();
};

#endif /* INC_NOTIFICATION_H_ */