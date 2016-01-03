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
    
    #Precti
    def do_precti(self, text):
    	"Precti text ulozeny v UTF-8 souboru na lokalnim disku Robika"
    	self.pub.publish("precti %s" % text)
    	ans(OK)

    OPERATION_HEAD_POSE= 1
    OPERATION_SET_PARKING_PHASE= 2
    OPERATION_SET_LED= 3
    OPERATION_SET_DPIN= 4
    OPERATION_SET_ARMPOWER= 5

    #zapni/vypni
    def do_zapni(self, text):
        "Zapni napajeni ruky"
        op = None
        if text == "ruku":
            op = self.OPERATION_SET_ARMPOWER
        elif text == "svetlo":
            op = self.OPERATION_SET_LED

        if op == None:
            ans("Co chces zapnout?")
        else:
            genctrl_msg = GenericControl()
            genctrl_msg.gen_operation = op
            genctrl_msg.gen_param1 = 1
            self.pub_genctrl.publish(genctrl_msg)
            ans(OK)

    def do_vypni(self, text):
        "Vypni napajeni ruky"
        op = None
        if text == "ruku":
            op = self.OPERATION_SET_ARMPOWER
        elif text == "svetlo":
            op = self.OPERATION_SET_LED

        if op == None:
            ans("Co chces vypnout?")
        else:
            genctrl_msg = GenericControl()
            genctrl_msg.gen_operation = op
            genctrl_msg.gen_param1 = 0
            self.pub_genctrl.publish(genctrl_msg)
            ans(OK)

    def complete_zapni(self, text, line, begidx, endidx):
        print line
        print begidx
        print endidx
        if text.startswith("s", 0, 1): print "svetlo"
        if text.startswith("r", 0, 1): print "ruku"

    #pokus s mluvenim pomoci action serveru
    def do_x(self, text):
        client = actionlib.SimpleActionClient('sound_play', SoundRequestAction)
        client.wait_for_server()
        goal = SoundRequestGoal()
        goal.sound_request.sound = SoundRequest.SAY
        goal.sound_request.command = SoundRequest.PLAY_ONCE
        goal.sound_request.arg = "To ti ale dalo zabrat co?"
        client.send_goal(goal)
        print "Goal sent"
        client.wait_for_result()
        print client.get_result()

    #General AI command
    def do_ai(self, text):
        "Obecny prikaz pro Robik AI, ktery neni podporovany timto shellem. Syntax: ai <prikaz>. Napr: ai rekni Kuba jede na kole"
        self.pub.publish(text)


    #Connect Wiimote
    def do_wiimote(self, line):
        "Pro pripojeni Wiimote ovladace, stiskni cervene tlacitko pod krytem baterie a spust prikaz wiimote"
        node = roslaunch.core.Node("wiimote", "wiimote_node.py")
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        process = launch.launch(node)
        ans(process.is_alive())
    
    #Cheat Sheet
    CASTE = [
    	'rekni Kuba jede na kole a vesele si zpiva',
    	'zapni svetlo'
    ]
    
    def do_caste(self, line):
    	"Seznam casto uzitych prikazu"
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
    
    def do_konec(self, line):
        "Ukoncit tuto konzoli"
        return True
    def do_EOF(self, line):
        "Ukoncit tuto konzoli"
        return True


if __name__ == '__main__':
    rospy.init_node('robik_shell')
    RobikShell().cmdloop()

