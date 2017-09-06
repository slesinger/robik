#!/usr/bin/python

import cmd
import roslib; roslib.load_manifest('robik')
import rospy
import roslaunch
from std_msgs.msg import String
import actionlib
from sound_play.msg import SoundRequest, SoundRequestAction, SoundRequestGoal
from subprocess import call
from robik.msg import GenericControl
import pprint

OK = "OK"

def ans(text):
    print "Robik> " + str(text)

class RobikShell(cmd.Cmd):
    """Robik interactive console."""

    def empty_line(self):
        return
    
    intro = "Robik interactive shell v1"
    prompt = "You> "
    emptyline = empty_line;
    
    pub = rospy.Publisher('robik_ai', String, queue_size=0)
    pub_genctrl = rospy.Publisher('robik_generic_control', GenericControl, queue_size=0)
    
    def do_shell(self, text):
        arg = ""  #TODO
        print(text.split(' ', 1)[0])
        call([text, arg])

    #Rekni
    def do_rekni(self, text):
    	"Hlasovy vystup"
    	self.pub.publish("rekni %s" % text)
    	ans(OK)
    
    def do_say(self, text):
    	"Voice output"
    	self.pub.publish("say %s" % text)
    	ans(OK)
    
    #Precti
    def do_precti(self, text):
    	"Precti cesky text ulozeny v UTF-8 souboru na lokalnim disku Robika"
    	self.pub.publish("precti %s" % text)
    	ans(OK)

    def do_read(self, text):
    	"Read english text file stored on Robik's filesystem"
    	self.pub.publish("read %s" % text)
    	ans(OK)

    #Uloz mapu
    def do_save_map(self, text):
    	"Save current map in map_server to <robik>/maps. Give map name as parameter"
    	call(["rosrun", "map_server" ,"map_saver" ,"-f" ,"/home/honza/workspace/src/robik/maps/%s" % text])
    	ans(OK)

    #Log
    def do_log(self, text):
	"Output latest log file"
	call(["tail" ,"-f", "/tmp/robik.log"])
	ans(OK)

    OPERATION_HEAD_POSE= 1
    OPERATION_SET_PARKING_PHASE= 2
    OPERATION_SET_LED= 3
    OPERATION_SET_DPIN= 4
    OPERATION_SET_ARMPOWER= 5

    #zapni/vypni
    def do_enable(self, text):
        "Enable someting"
        op = None
        if text == "arm":
            op = self.OPERATION_SET_ARMPOWER
        elif text == "light":
            op = self.OPERATION_SET_LED

        if op == None:
            ans("What do you want to enable? arm light")
        else:
            genctrl_msg = GenericControl()
            genctrl_msg.gen_operation = op
            genctrl_msg.gen_param1 = 1
            self.pub_genctrl.publish(genctrl_msg)
            ans(OK)

    def do_disable(self, text):
        "Disable something"
        op = None
        if text == "arm":
            op = self.OPERATION_SET_ARMPOWER
        elif text == "light":
            op = self.OPERATION_SET_LED

        if op == None:
            ans("What do you want to disable?")
        else:
            genctrl_msg = GenericControl()
            genctrl_msg.gen_operation = op
            genctrl_msg.gen_param1 = 0
            self.pub_genctrl.publish(genctrl_msg)
            ans(OK)

    def complete_enable(self, text, line, begidx, endidx):
        if text == "": print "arm light"
        if text.startswith("l", 0, 1): print "light"
        if text.startswith("a", 0, 1): print "arm"

    #pokus s mluvenim pomoci action serveru
    def do_x(self, text):
        client = actionlib.SimpleActionClient('sound_play', SoundRequestAction)
        client.wait_for_server()
        goal = SoundRequestGoal()
        goal.sound_request.sound = SoundRequest.SAY
        goal.sound_request.command = SoundRequest.PLAY_ONCE
        goal.sound_request.arg = "Blah blah I am saying something"
        client.send_goal(goal)
        print "Goal sent"
        client.wait_for_result()
        print client.get_result()

    #General AI command
    def do_ai(self, text):
        "Give a general command to Robik AI that is not supported by this shell. Syntax: ai <command>. E.g.: ai rekni Kuba jede na kole"
        self.pub.publish(text)


    #Connect Wiimote
    def do_wii(self, line):
        "For wii remote connection press red button inside the controller and run wiimote command in this shell."
	do_wiimote(self, line)

    def do_wiimote(self, line):
        "For wii remote connection press red button inside the controller and run wiimote command in this shell."
        node = roslaunch.core.Node("wiimote", "wiimote_node.py")
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        process = launch.launch(node)
        ans(process.is_alive())


    #Arm related stuff
    def do_arm(self, text):
        "Execute arm movements. Use arm goal name as parameter. Goals are defined in this shell temporarily."

	if not text == "":
            self.pub.publish("arm " + text)
        else:
            ans("Give me goal name for the arm.")

    def complete_arm(self, text, line, begidx, endidx):
        if text == "": print "home open close grab handover up "
        if text.startswith("ho",0, 2): print "home"
        if text.startswith("o", 0, 1): print "open"
        if text.startswith("c", 0, 1): print "close"
        if text.startswith("g", 0, 1): print "grab"
        if text.startswith("ha",0, 2): print "handover"
        if text.startswith("u", 0, 1): print "up"


    #Cheat Sheet
    CASTE = [
    	'rekni Kuba jede na kole a vesele si zpiva',
	'say Better safe than sorry',
    	'enable light'
    ]
    
    def do_quickref(self, line):
    	"Frequently used commands:"
    	if line:
    		try:
    			i = int(line)
    		except:
    			line = None
    	
    	if line:
    		self.pub.publish(self.CASTE[int(line)])
    		ans(OK)
    	else:
	    	for c in self.CASTE:
    			print str(self.CASTE.index(c)) + ' ' + c
    
    def do_quit(self, line):
        "Exit from this shell"
        return True
    def do_EOF(self, line):
        "Exit this shell"
        return True


if __name__ == '__main__':
    rospy.init_node('robik_shell')
    RobikShell().cmdloop()

