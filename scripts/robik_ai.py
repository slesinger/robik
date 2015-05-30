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
import roslib; roslib.load_manifest('robik')
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


audio_args = namedtuple('audio_args',['language','output'])

def audio_extract(input_text='',args=None):
    # This accepts :
    #   a dict,
    #   an audio_args named tuple
    #   or arg parse object
    if args is None:
        args = audio_args(language='en',output=open('output.mp3', 'w'))
    if type(args) is dict:
        args = audio_args(
                    language=args.get('language','en'),
                    output=open(args.get('output','output.mp3'), 'w')
        )
    #process input_text into chunks
    #Google TTS only accepts up to (and including) 100 characters long texts.
    #Split the text in segments of maximum 100 characters long.
    combined_text = split_text(input_text)

    #download chunks and write them to the output file
    for idx, val in enumerate(combined_text):
        mp3url = "http://translate.google.com/translate_tts?tl=%s&q=%s&total=%s&idx=%s" % (
            args.language,
            urllib.quote(val),
            len(combined_text),
            idx)
        headers = {"Host": "translate.google.com",
                   "Referer": "http://www.gstatic.com/translate/sound_player2.swf",
                   "User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_7_3) "
                                 "AppleWebKit/535.19 (KHTML, like Gecko) "
                                 "Chrome/18.0.1025.163 Safari/535.19"
        }
        req = urllib2.Request(mp3url, '', headers)
        sys.stdout.write('.')
        sys.stdout.flush()
        if len(val) > 0:
            try:
                response = urllib2.urlopen(req)
                args.output.write(response.read())
                time.sleep(.5)
            except urllib2.URLError as e:
                print ('%s' % e)
    args.output.close()
    print('Saved MP3 to %s' % args.output.name)







def say(text):
	rospy.loginfo("I say: %s", text)
	soundhandle.say(text.decode('utf8').encode('iso-8859-2'), "voice_czech_ph")
	#audio_extract(input_text='tunnel snakes rule apparently', args = {'language':'en','output':'/tmp/outputto.mp3'})

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
	rospy.loginfo("I heard %s",data.data)
	
	tokens = data.data.split(" ", 1)
	token_1 = tokens[0]

	if token_1 == 'map':
		print "action map"
		
	if token_1 == 'zvedni':
		print "zvedni"
		
	if token_1 == 'poloz':
		print "poloz"

	if token_1 == 'zaparkuj ruku':
		zaparkuj_ruku()

	if token_1 == 'zaparkuj se':
		zaparkuj_ruku()

	if token_1 == 'rekni':
		say(tokens[1])

        if token_1 == 'menu':
                menu(tokens[1])
		
def zaparkuj_ruku():
	client = actionlib.SimpleActionClient('robik_action_arm', robik.msg.armAction)
	client.wait_for_server()
	goal = robik.msg.armGoal(command=1, opTimeMs=1000)
	client.send_goal(goal)
	client.wait_for_result(rospy.Duration.from_sec(5.0))

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
	zaparkuj_ruku()
	if poweroff == True:
		vypniSe()

def bored():
	say('S lidma je nuda. Postavte mi ještě kamaráda')

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
	say("vždy k službám")

	rospy.Timer(rospy.Duration(3000), seNudim)

	rospy.spin()

def signal_handler(signal, frame):
	rospy.loginfo('AI shutdown')
	zaparkuj()
	time.sleep(3)
	rospy.loginfo('AI shutdown completed or timeout')

if __name__ == '__main__':
	try:
		signal.signal(signal.SIGINT, signal_handler)
		AI()
	except rospy.ROSInterruptException:
		pass

