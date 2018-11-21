#!/usr/bin/env python

from unr_deepspeech.srv import *
import rospy
from rospkg import RosPack

import wave
import struct

from deepspeech_node import DeepspeechNode

import sys
import glob
import os

def handle_listener(req):

    global listener
    
    audio_path = req.filename
    audio_file = wave.open(audio_path)
    fs = audio_file.getframerate()
    audio_string = audio_file.readframes(-1)
    audio = [struct.unpack("<h", audio_string[i:i+2])[0]
             for i in xrange(0, len(audio_string), 2)]    

    text = listener.stt(fs, audio)
    print(text)
    
    return text

def listener_server():
    global listener

    rp = RosPack()
    package_path = rp.get_path("unr_deepspeech") 
    model_path = "{}/model".format(package_path)
    if rospy.has_param("/unr_deepspeech/model"):
        model_path = rospy.get_param("/unr_deepspeech/model")
    print "Loading model from" + model_path
    
    listener = DeepspeechNode(model_path=model_path)
    
    if rospy.has_param("/unr_deepspeech/dictionary"):
        dict_filename = rospy.get_param("unr_deepspeech/dictionary")
        dict_path = package_path + "/" + dict_filename
        print "Loading dictionary from " + dict_path
        with open(dict_path) as dict_file:
            listener.dictionary = dict_file.read().split("\n")

    if rospy.has_param("/unr_deepspeech/possibilities"):
        possibilities_path = rospy.get_param("unr_deepspeech/possibilities")
        print "Loading possible transcripts from " + possibilities_path
        with open(possibilities_path) as possibilities_file:
            listener.possibilities = possibilities_file.read().split("\n")

    rospy.init_node('listener_server')

    s = rospy.Service('listen', Listen, handle_listener)
    print "Ready to interpret speech audio."
    rospy.spin()

if __name__ == "__main__":
    listener_server()
