#!/usr/bin/env python

import rospy
import time
import pygame
from std_msgs.msg import UInt8
from ema_common_msgs.srv import Display, DisplayResponse


global screen
global myfont
global scale
global chars 
global lines 
global scale 
global size 


chars = 16
lines = 2
scale = 3
size = [12*chars*scale,20*lines*scale]

pygame.init()
myfont = pygame.font.SysFont("monospace", 20*scale)
screen= pygame.display.set_mode(size)
pygame.display.set_caption("LCD %dx%d      Mouse right: [+] | Mouse left: [-]"%(chars,lines))


def screen_clear():
    global scale
    global size
    
    for pos in range(1,size[0],12*scale):
        for line in range (0,lines):
            pygame.draw.rect(screen, pygame.Color("green"), pygame.Rect(pos, 20*scale*line+2, 12*scale-1, 20*scale-4))
    pygame.display.flip()

def draw_text(text,line,position):
    global myfont
    global scale
    global size
    global screen
    step = 12*scale
    start = step*position
    for pos in range(start+1,step*len(text)+start,step):
        pygame.draw.rect(screen, pygame.Color("green"), pygame.Rect(pos, 20*scale*line+2, 12*scale-1, 20*scale-4))
    
    text_object= myfont.render(text, 2*scale, (0,0,0))
    screen.blit(text_object, (start, 20*line*scale))
    pygame.display.flip()

def display_request(req):
    
    if req.clear:
        screen_clear()
    draw_text(req.message, req.line,req.position)
    return DisplayResponse(success=True)

        
def main():
    # init display node
    
    rospy.init_node('display_emulator', anonymous=False)
    display_service =rospy.Service('display/write', Display, display_request)
    pub = rospy.Publisher('button/action', UInt8, queue_size=10)
    screen_clear()
    draw_text("Pronto para Uso", 0,1)

    freq=20
    rate = rospy.Rate(freq)
    wait_time = 200
    
    current_value = 0
    button_press=False
    
    last = 0
    done = False
    while not done and not rospy.is_shutdown():
        if button_press:
            if (time.time()-last) * 1000 > wait_time:
                pub.publish(UInt8(current_value))
                current_value = 0
                button_press=False

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
                rospy.signal_shutdown("Closed")
            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse=pygame.mouse.get_pressed()
                pressed=0b00000000
                if mouse[0]:
                    pressed |= 0b00000001
                if mouse[2]:
                    pressed |= 0b00000010
                current = time.time()
                if current_value == 0:
                    current_value = pressed
                elif current_value == 1:
                    if pressed == 1:
                        current_value = 4
                    elif pressed == 2:
                        current_value = 3
                elif current_value == 2:
                    if pressed == 1:
                        current_value = 3
                    elif pressed == 2:
                        current_value = 5
                button_press = True
                last = current

        rate.sleep()                  
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
