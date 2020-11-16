#!/usr/bin/env python3

# Copyright (c) 2020, FADA-CATEC

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Helper module for python thrift interface to ZLP Service. 
This module contains utility classes and methods which ease the usage of the thrift interface to ZLP Service."""


class KeyboardParameters(object):
    """."""

    def __init__(self):
        """."""
        from pynput import keyboard

        self.KEY_UP     = {keyboard.Key.up}
        self.KEY_DOWN   = {keyboard.Key.down}
        self.KEY_LEFT   = {keyboard.Key.left}
        self.KEY_RIGHT  = {keyboard.Key.right}
        self.KEY_PLUS   = {keyboard.KeyCode(char='+')}
        self.KEY_MINUS  = {keyboard.KeyCode(char='-')}
        self.CTRL_LEFT  = {keyboard.Key.left, keyboard.Key.ctrl}
        self.CTRL_RIGHT = {keyboard.Key.right, keyboard.Key.ctrl}
        self.ESC        = {keyboard.Key.esc}

        self.COMBINATIONS = [self.KEY_UP,self.KEY_UP,self.KEY_DOWN,self.KEY_LEFT,self.KEY_RIGHT,
                            self.KEY_PLUS,self.KEY_MINUS,self.CTRL_LEFT,self.CTRL_RIGHT,self.ESC]

class KeyboardControl(object):
    """This class is used to monitor the keyboard presses."""

    def __init__(self,projector_client,projection_element):
        """Initialize the KeyboardControl object.

        Args:
            projector_client (object):
            projection_element (object):
        """
        self.keyboard_params = KeyboardParameters()

        self.projector_client = projector_client
        self.projection_element = projection_element

        self.current = set()

    def on_press(self,key,coord_sys, proj_elem_params):
        """.
        
        Args:
            key (): 
        """
        if any([key in COMBO for COMBO in self.keyboard_params.COMBINATIONS]):
            self.current.add(key)

            if self.current == self.keyboard_params.KEY_UP:
                print ("KEY_UP")
                self.projection_element.translate_figure(proj_elem_params,0,1,0)
                self.projector_client.update_project(coord_sys)
            elif self.current == self.keyboard_params.KEY_DOWN:
                print ("KEY_DOWN")
                self.projection_element.translate_figure(proj_elem_params,0,-1,0)
                self.projector_client.update_project(coord_sys)
            elif self.current == self.keyboard_params.KEY_LEFT:
                print ("KEY_LEFT")
                self.projection_element.translate_figure(proj_elem_params,-1,0,0)
                self.projector_client.update_project(coord_sys)
            elif self.current == self.keyboard_params.KEY_RIGHT:
                print ("KEY_RIGHT")
                self.projection_element.translate_figure(proj_elem_params,1,0,0)
                self.projector_client.update_project(coord_sys)
            elif self.current == self.keyboard_params.KEY_PLUS:
                print ("KEY_PLUS")
                self.projection_element.scale_figure(proj_elem_params,2)
                self.projector_client.update_project(coord_sys)
            elif self.current == self.keyboard_params.KEY_MINUS:
                print ("KEY_MINUS")
                self.projection_element.scale_figure(proj_elem_params,0.5)
                self.projector_client.update_project(coord_sys)
            elif self.current == self.keyboard_params.CTRL_LEFT:
                print ("CTRL_LEFT")
                self.projection_element.rotate_figure(proj_elem_params,0,0,1)
                self.projector_client.update_project(coord_sys)
            elif self.current == self.keyboard_params.CTRL_RIGHT:
                print ("CTRL_RIGHT")
                self.projection_element.rotate_figure(proj_elem_params,0,0,-1)
                self.projector_client.update_project(coord_sys)
            elif self.current == self.keyboard_params.ESC:
                print ("ESC")
                self.projector_client.stop_project()

    def on_release(self,key):
        """.
        
        Args:
            key (): 
        """
        if any([key in COMBO for COMBO in self.keyboard_params.COMBINATIONS]):
            
            if self.current == self.keyboard_params.ESC:
                self.current.remove(key)
                return False

            self.current.remove(key)
            
    def init_keyboard_listener(self,cs,figure_params):
        """."""
        from pynput import keyboard
        
        try:
            with keyboard.Listener(on_press=lambda event: self.on_press(event, coord_sys=cs, proj_elem_params=figure_params),
                        on_release=self.on_release) as listener:
                listener.join()

            success = True
            message = "Keyboard monitoring finished correctly."

        except Exception as e:
            success = False
            message = e

        return success,message