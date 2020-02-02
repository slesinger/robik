#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Robik AI.
	parameters:
		~none - none
	publications:
		~goal (std_msgs/String) - action goals
	services:
"""

import roslib; roslib.load_manifest('sound_play')
import roslib; roslib.load_manifest('robik_basestation')
import robik
import robik.msg
import rospy, os, sys
import actionlib
import random
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
from random import choice
import signal
import sys
import time
import re
from collections import namedtuple
import moveit_msgs.msg
import moveit_commander
import json
from watson_developer_cloud import ConversationV1


conversation = ConversationV1(
    username='d3ffcc1b-14bd-4ac7-a8e6-d3d9ea21cad7',
    password='MnAHpjMePuWu',
    version='2017-08-17')

workspace_id = '6667875c-eca9-4568-85c7-8fd88a5487d9'

response = conversation.message(
    workspace_id=workspace_id,
    message_input={'text': ''})
context = response['context']


def split_text(input_text, max_length=100):
    """
    Try to split between sentences to avoid interruptions mid-sentence.
    Failing that, split between words.
    See split_text_rec
    """
    def split_text_rec(input_text, regexps, max_length=max_length):
        """
        Split a string into substrings which are at most max_length.
        Tries to make each substring as big as possible without exceeding
        max_length.
        Will use the first regexp in regexps to split the input into
        substrings.
        If it it impossible to make all the segments less or equal than
        max_length with a regexp then the next regexp in regexps will be used
        to split those into subsegments.
        If there are still substrings who are too big after all regexps have
        been used then the substrings, those will be split at max_length.
        Args:
            input_text: The text to split.
            regexps: A list of regexps.
                If you want the separator to be included in the substrings you
                can add parenthesis around the regular expression to create a
                group. Eg.: '[ab]' -> '([ab])'
        Returns:
            a list of strings of maximum max_length length.
        """
        if(len(input_text) <= max_length): return [input_text]

        #mistakenly passed a string instead of a list
        if isinstance(regexps, basestring): regexps = [regexps]
        regexp = regexps.pop(0) if regexps else '(.{%d})' % max_length

        text_list = re.split(regexp, input_text)
        combined_text = []
        #first segment could be >max_length
        combined_text.extend(split_text_rec(text_list.pop(0), regexps, max_length))
        for val in text_list:
            current = combined_text.pop()
            concat = current + val
            if(len(concat) <= max_length):
                combined_text.append(concat)
            else:
                combined_text.append(current)
                #val could be >max_length
                combined_text.extend(split_text_rec(val, regexps, max_length))
        return combined_text

    return split_text_rec(input_text.replace('\n', ''),
                          ['([\,|\.|;]+)', '( )'])






def say(text, lang='en'):
	rospy.loginfo("I say ({}): %s".format(lang), text)
	if lang == 'cs':
		soundhandle.say(text.decode('utf8').encode('iso-8859-2'), "voice_czech_ph")
	else:
		soundhandle.say(text)

def say_action(text, lang='en'):
	rospy.loginfo("I say: %s", text)
	if lang == 'cs':
		soundhandle.say(text.decode('utf8').encode('iso-8859-2'), "voice_czech_ph")
	else:
		soundhandle.say(text)


def read(text, lang='en'):
    f = open(text, 'r')
    complete = ""
    for para in f:
        complete += para
        #for s in split_text(para, 200):  #problem here is that reading is asynchrnous and hence second paragraph is commanded to be read right after first one starts (not finishes)
    say_action(complete, lang)


def jakSeMas():
	say("Zatížení je " + str(int(os.getloadavg()[0] * 100)) + " procent")
	time.sleep(3)
	sit_status = os.popen('sudo nmcli c status | grep wlan0').read()
	sit_nova = sit_status.partition(' ')[0]
	say("Síť přepnuta na " + sit_nova)

def zaparkujSeANabijSe():
	say("Jdu se zaparkovat a napapkat")

def jenSeZaparkuj():
	say("Jdu se zaparkovat")
	zaparkuj_se()

def jenSeNabij():
	say("Tak já se teda nabiju, ale jen jestli jsem v domečku")
	nabij_se()

def vypniSe():
	say("Tak čau. Jdu se vyspinkat")
	os.system("sudo poweroff")

def prepniSit(sit):
	say("Přepínám síť na " + sit)
	os.system("sudo nmcli c up id " + sit)
	say("Počkej chvilku")
	time.sleep(5)
	jakSeMas()

#Arm
def handleArm(text):
  global arm
  global moveit_commander

  if text == "home":
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_named_target("home")
    arm.plan()
    arm.go()
  elif text == "open":
    arm = moveit_commander.MoveGroupCommander("gripper")
    arm.set_named_target("open")
    arm.plan()
    arm.go()
  elif text == "close":
    arm = moveit_commander.MoveGroupCommander("gripper")
    arm.set_named_target("close")
    arm.plan()
    arm.go()
  elif text == "state":
    print arm.get_current_state()
  elif text == "grab":
    say("OK. I will give you some sweets.")
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_named_target("inbowl1")
    arm.plan()
    arm.go()
    rospy.sleep(1)
    arm = moveit_commander.MoveGroupCommander("gripper")
    arm.set_named_target("open")
    arm.plan()
    arm.go()
    rospy.sleep(1)
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_named_target("inbowl2")
    arm.plan()
    arm.go()
    rospy.sleep(1)
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_named_target("inbowl3")
    arm.plan()
    arm.go()
    rospy.sleep(1)
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_named_target("inbowl4")
    arm.plan()
    arm.go()
    rospy.sleep(1)
    arm = moveit_commander.MoveGroupCommander("gripper")
    arm.set_named_target("close")
    arm.plan()
    arm.go()
    rospy.sleep(1)
    arm.set_named_target("close")
    arm.plan()
    arm.go()
    rospy.sleep(1)
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_named_target("inbowl1")
    arm.plan()
    arm.go()
    rospy.sleep(2)
    arm.set_named_target("handover")
    arm.plan()
    arm.go()
    say("Get ready to take the sweets")
    rospy.sleep(2)
    arm = moveit_commander.MoveGroupCommander("gripper")
    arm.set_named_target("open")
    arm.plan()
    arm.go()
    rospy.sleep(1)
    arm.set_named_target("close")
    arm.plan()
    arm.go()
    rospy.sleep(1)
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_named_target("home")
    arm.plan()
    arm.go()
  elif text == "wave":
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_named_target("waveleft")
    arm.plan()
    arm.go()
    say("Bye Bye. Have fun and get back to us soon.")
    rospy.sleep(1)
    arm.set_named_target("waveright")
    arm.plan()
    arm.go()
    arm.set_named_target("waveleft")
    arm.plan()
    arm.go()
    rospy.sleep(1)
    arm.set_named_target("waveright")
    arm.plan()
    arm.go()
    rospy.sleep(1)
    arm.set_named_target("home")
    arm.plan()
    arm.go()


def handle_speech(text):
    global context

    print "USER  < {}".format(text.data)
    response = conversation.message(
        workspace_id=workspace_id,
        message_input={'text': text.data},
        context=context)

    context = response['context']
    for rl in response['output']['text']:
		tokens = rl.split('|')
		for t in tokens:
			if t.startswith('{'):
				handleCommand(t[1:-1])
			else:
				say(rl)


#Menu
menu_items = {1:(4,2,"Jak se máš", jakSeMas),
              2:(1,3,"Zaparkuj se", 21),
               21:(23,22,"Zaparkuj a nabij se", zaparkujSeANabijSe),
               22:(21,23,"Jen se zaparkuj", jenSeZaparkuj),
               23:(22,21,"Jen se nabij", jenSeNabij),
              3:(2,4,"Vypni se", vypniSe),
              4:(3,1,"Přepni síť", 41),
               41:(43,42,"Přepni na stromeček", prepniSit, "stromecek"),
               42:(41,43,"Přepni na stromeček nula", prepniSit, "stromecek0"),
               43:(42,41,"Přepni na asus", prepniSit, "asus")
}


menu_current = 0
menu_breadcrumbs = []


def menu_say(nextMenu):
        global menu_current, menu_items
        menu_current = nextMenu
        say(menu_items[nextMenu][2])

def menu(button):
        global menu_current

        if button == 'menu' and menu_current == 0:  #enable menu
		say("Tak co to bude")
		time.sleep(2)
                menu_say(1)

	elif button == 'menu' and menu_current != 0:  #disable menu
		say("Tak nic no")
                menu_current = 0

	elif button == 'up' and menu_current != 0:
		menu_say(menu_items[menu_current][0])

	elif button == 'down' and menu_current != 0:
		menu_say(menu_items[menu_current][1])

	elif button == 'ok' and menu_current != 0:
		action = menu_items[menu_current][3]
		if type(action) is int:
			menu_breadcrumbs.append(menu_current)
			menu_say(action)
		else:
			if len(menu_items[menu_current]) == 4:
				menu_items[menu_current][3]()
			else:
				menu_items[menu_current][3](menu_items[menu_current][4])

	elif button == 'cancel' and menu_current != 0:
		if len(menu_breadcrumbs) > 0:
			menu_current = menu_breadcrumbs.pop()
			menu_say(menu_current)

def recognitionCallback(data):
	handleCommand(data.data)

def handleCommand(cmd):
	rospy.loginfo("AI command %s", cmd)

	tokens = cmd.split(" ", 1)
	token_1 = tokens[0]

	if token_1 == 'map':
		print "action map"

	elif token_1 == 'zvedni':
		print "zvedni"

	elif token_1 == 'poloz':
		print "poloz"

	elif token_1 == 'rekni':
		say(tokens[1], lang='cs')

	elif token_1 == 'say':
		say(tokens[1])

	elif token_1 == 'precti':
		read(tokens[1], lang='cs')

	elif token_1 == 'read':
		read(tokens[1])

        elif token_1 == 'menu':
                menu(tokens[1])

        elif token_1 == 'arm':
                handleArm(tokens[1])


def zaparkuj_se():
	client = actionlib.SimpleActionClient('robik_action_move', robik.msg.moveAction)
	client.wait_for_server()
	goal = robik.msg.moveGoal(command=9)
	client.send_goal(goal)
	client.wait_for_result(rospy.Duration.from_sec(50.0))

def nabij_se():
	rospy.loginfo("nabij_se()")
	client = actionlib.SimpleActionClient('robik_action_move', robik.msg.moveAction)
	client.wait_for_server()
	goal = robik.msg.moveGoal(command=10)
	client.send_goal(goal)
	client.wait_for_result(rospy.Duration.from_sec(5.0))

def shut_down(poweroff):
	if poweroff == True:
		vypniSe()

def bored():
	say('Hallo. I am getting real bored. Please create me some new friend.')

def seNudim(event):
	from random import choice
	actions = [bored]
	random.choice(actions)()

def AI():
	""" Initialize components. """
	global soundhandle

	soundhandle = SoundClient()
	rospy.loginfo("Starting AI.")
	rospy.init_node('robik_ai');
	rospy.Subscriber("robik_ai", String, recognitionCallback)
	rospy.Subscriber("/gspeech/speech", String, handle_speech)
	moveit_cmd = moveit_commander.RobotCommander()
    	moveit_scene = moveit_commander.PlanningSceneInterface()
    	moveit_display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=5)


	say("ehm. ehm. Robik is ready at your service.")

	rospy.Timer(rospy.Duration(3000), seNudim)

	rospy.spin()

	time.sleep(2)
	handleCommand("arm home")


def signal_handler(signal, frame):
	rospy.loginfo('AI shutdown')
	time.sleep(3)
	rospy.loginfo('AI shutdown completed or timeout')

if __name__ == '__main__':
	try:
		signal.signal(signal.SIGINT, signal_handler)
		AI()
	except rospy.ROSInterruptException:
		pass
