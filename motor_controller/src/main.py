import rospy
    
def set_k(settings):
    print("PID settings changed from ", PIDModel.kp, PIDModel.ki, PIDModel.kd,
          " to ", settings.Kp, settings.Ki, settings.Kd)
    PIDModel.kp = settings.Kp
    PIDModel.ki = settings.Ki
    PIDModel.kd = settings.Kd

def main():
    rospy.Subscriber("/PID_settings", PID_settings, self.set_k, queue_size=1)

if __name__ == '__main__':
    main()
