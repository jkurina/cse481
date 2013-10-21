#!/usr/bin/env python

import json
import os

class ArmDB:

    instance = None

    class Singleton:

        positions = None
        dbFile = None
        filePath = (str(os.path.dirname(os.path.realpath(__file__))) +
                "/arm_pos.db")
        
        def __init__(self):
            if self.positions == None:
                self.loadPos()

        def loadPos(self):
            try:
                self.dbFile = open(self.filePath, 'r')
                content = self.dbFile.read()
                if content:
                    self.positions = json.loads(content)
                self.dbFile.close()
            except IOError:
                self.positions = {"r":{}, "l":{}}

        def savePos(self, side_prefix, name, position):
            self.checkSidePrefix(side_prefix)
            if name in self.positions[side_prefix]:
                return False
            self.positions[side_prefix][name] = position
            self.dbFile = open(self.filePath, 'w')
            self.dbFile.write(json.dumps(self.positions))
            self.dbFile.close()
            return True

        def rmPos(self, side_prefix, name):
            self.checkSidePrefix(side_prefix)
            if name not in self.positions[side_prefix]:
                return False
            self.positions[side_prefix].pop(name, None)
            self.dbFile = open(self.filePath, 'w')
            self.dbFile.write(json.dumps(self.positions))
            self.dbFile.close()
            return True
        
        def checkSidePrefix(self, side_prefix):
            if side_prefix not in ['l', 'r']:
                raise Exception("side_prefix has to be either 'l' or 'r'")

        def getAllLeftPos(self):
            return self.positions["l"]

        def getAllRightPos(self):
            return self.positions["r"]
          
    def __init__(self):
        if ArmDB.instance is None:
            ArmDB.instance = ArmDB.Singleton()

    '''
    eg, savePos('r', 'position name', get_joint_state())
    get_join_state() is the one from arm_gui.py
    This function will return true on success, false if the 
    name already exists
    '''
    def savePos(self, side_prefix, name, position):
        return ArmDB.instance.savePos(side_prefix, name, position)

    '''return true on success, otherwise false'''
    def rmPos(self, side_prefix, name):
        return ArmDB.instance.rmPos(side_prefix, name)

    # return a dict, from name to position(array)
    def getAllLeftPos(self):
        return ArmDB.instance.getAllLeftPos()

    def getAllRightPos(self):
        return ArmDB.instance.getAllRightPos();
