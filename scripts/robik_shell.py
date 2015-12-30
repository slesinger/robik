#!/usr/bin/python

import cmd
import roslib; roslib.load_manifest('robik')
import rospy
from std_msgs.msg import String

OK = "(Robik) OK"

class RobikShell(cmd.Cmd):
    """Robik interactive console."""
    prompt = "( You ) "
    
    pub = rospy.Publisher('robik_ai', String, queue_size=10)
    
    #Greet
    FRIENDS = [ 'Alice', 'Adam', 'Barbara', 'Bob' ]
    
    def do_greet(self, person):
        "Greet the person"
        if person and person in self.FRIENDS:
            greeting = 'hi, %s!' % person
        elif person:
            greeting = "hello, " + person
        else:
            greeting = 'hello'
        print greeting
    
    def complete_greet(self, text, line, begidx, endidx):
        if not text:
            completions = self.FRIENDS[:]
        else:
            completions = [ f
                            for f in self.FRIENDS
                            if f.startswith(text)
                            ]
        return completions
    
    
    #Rekni
    def do_rekni(self, text):
    	"Hlasovy vystup"
    	self.pub.publish("rekni %s" % text)
    	print OK
    
    
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
    		print OK
    	else:
	    	for c in self.CASTE:
    			print str(self.CASTE.index(c)) + ' ' + c
    
    def do_EOF(self, line):
        return True

if __name__ == '__main__':
    rospy.init_node('robik_shell')
    RobikShell().cmdloop()

