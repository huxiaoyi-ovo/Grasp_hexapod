import pygame
import time

class JoyStick:
    def __init__(self):
        _i=0
        pygame.init()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        self.last_button_time=time.time()
        while joystick_count == 0:
            for evt in pygame.event.get():
                if evt.type == pygame.JOYDEVICEADDED:
                    joystick_count = pygame.joystick.get_count()
                    break
            if _i == 0:
                print("\033[93m Waiting for joystick to connect... \033[0m")
                _i=1
            time.sleep(0.2)
        self.joystick=pygame.joystick.Joystick(0)
        self.joystick.init()
        print("\033[92m ------->" + self.joystick.get_name() +" connected<------ \033[0m")

        self.reset=0.0 #buttons B
        self.vx=0.0
        self.vy=0.0
        self.vz=0.0
        self.yaw=0.0
    
        
    def get_commands(self):
        """
        @output: reset, vx vy vz yaw 

        @ranges:reset: 0/1, others: -1~1
        """
        for evt in pygame.event.get():
            pass
        self.reset = self.joystick.get_button(1)
        if self.reset:
            if time.time()-self.last_button_time>1.0:
                self.reset=1.0
                self.last_button_time=time.time()
            else:
                self.reset=0.0
        self.vx = 0 if abs(self.joystick.get_axis(0))<0.1 else self.joystick.get_axis(0)
        self.vy = 0 if abs(self.joystick.get_axis(1))<0.1 else -self.joystick.get_axis(1)
        self.yaw = 0 if abs(self.joystick.get_axis(3))<0.1 else -self.joystick.get_axis(3)
        self.vz = 0 if abs(self.joystick.get_axis(4))<0.1 else -self.joystick.get_axis(4)
        return self.reset, self.vx, self.vy, self.vz, self.yaw

if __name__ == '__main__':
    joy_stick = JoyStick()
    while True:
        reset, vx, vy, vz, yaw = joy_stick.get_commands()
        print(type(reset))
        print(f"reset: {reset}, vx: {vx}, vy: {vy}, vz: {vz}, yaw: {yaw}")
        time.sleep(0.01)
