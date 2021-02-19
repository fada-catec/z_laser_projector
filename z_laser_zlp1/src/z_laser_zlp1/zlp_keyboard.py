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

"""This module contains utility classes and methods which wrap the keyboard listener for monitoring."""

class KeyboardParameters(object):
    """This class contains the accepted monitored keys.

    Attributes:
        KEY_UP (enum): keyboard key up
        KEY_DOWN (enum): keyboard key down
        KEY_LEFT (enum): keyboard key left
        KEY_RIGHT (enum): keyboard key right
        KEY_PLUS (enum): keyboard key +
        KEY_MINUS (enum): keyboard key -
        CTRL_LEFT (enum): keyboard key ctrl+let
        CTRL_RIGHT (enum): keyboard key ctrl+right
        ESC (enum): keyboard key esc
        COMBINATIONS (list): list of keyboard keys and combinations defined
    """
    def __init__(self):
        """Initialize the KeyboardParameters object."""
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
    """This class is used to monitor the keyboard presses in order to change properties of a projection 
    element in real time.
    
    Args:
        projector_client (object): object with the methods of projector client class
        projection_element (object): object with the methods of projection element class

    Attributes:
        keyboard_params (object): object with the accepted monitored keys
        projector_client (object): object with the methods of projector client class
        projection_element (object): object with the methods of projection element class
    """
    def __init__(self, projector_client, projection_element):
        """Initialize the KeyboardControl object."""
        self.keyboard_params = KeyboardParameters()

        self.projector_client = projector_client
        self.projection_element = projection_element

        self.current = set()

    def on_press(self, key, cs_name, proj_elem_params):
        """Check if the key pressed if one of the list and execute the respective tasks.
        
        Args:
            key (enum): key pressed
            cs_name (str): name of the coordinate system
            proj_elem_params (object): object with the parameters of the projection element to monitor
        """
        if any([key in COMBO for COMBO in self.keyboard_params.COMBINATIONS]):
            self.current.add(key)

            if self.current == self.keyboard_params.KEY_UP:
                print ("KEY_UP")
                self.projection_element.translate_figure(proj_elem_params,0,1,0)
                self.projector_client.update_project(cs_name)
            elif self.current == self.keyboard_params.KEY_DOWN:
                print ("KEY_DOWN")
                self.projection_element.translate_figure(proj_elem_params,0,-1,0)
                self.projector_client.update_project(cs_name)
            elif self.current == self.keyboard_params.KEY_LEFT:
                print ("KEY_LEFT")
                self.projection_element.translate_figure(proj_elem_params,-1,0,0)
                self.projector_client.update_project(cs_name)
            elif self.current == self.keyboard_params.KEY_RIGHT:
                print ("KEY_RIGHT")
                self.projection_element.translate_figure(proj_elem_params,1,0,0)
                self.projector_client.update_project(cs_name)
            elif self.current == self.keyboard_params.KEY_PLUS:
                print ("KEY_PLUS")
                self.projection_element.scale_figure(proj_elem_params,2)
                self.projector_client.update_project(cs_name)
            elif self.current == self.keyboard_params.KEY_MINUS:
                print ("KEY_MINUS")
                self.projection_element.scale_figure(proj_elem_params,0.5)
                self.projector_client.update_project(cs_name)
            elif self.current == self.keyboard_params.CTRL_LEFT:
                print ("CTRL_LEFT")
                self.projection_element.rotate_figure(proj_elem_params,0,0,1)
                self.projector_client.update_project(cs_name)
            elif self.current == self.keyboard_params.CTRL_RIGHT:
                print ("CTRL_RIGHT")
                self.projection_element.rotate_figure(proj_elem_params,0,0,-1)
                self.projector_client.update_project(cs_name)
            elif self.current == self.keyboard_params.ESC:
                print ("ESC")
                self.projector_client.stop_project()

    def on_release(self, key):
        """Remove current stored key, on release.
        
        Args:
            key (enum): key pressed
        """
        if any([key in COMBO for COMBO in self.keyboard_params.COMBINATIONS]):
            
            if self.current == self.keyboard_params.ESC:
                self.current.remove(key)
                return False

            self.current.remove(key)
            
    def init_keyboard_listener(self, cs_name, figure_params):
        """Start keyboard listener for monitoring key presses.
        
        Args:
            cs_name (str): name of the coordinate system
            figure_params (object): object with the parameters of the projection element to monitor
        """
        from pynput import keyboard
        
        try:
            with keyboard.Listener(on_press=lambda event: self.on_press(event, cs_name=cs_name, proj_elem_params=figure_params),
                        on_release=self.on_release) as listener:
                listener.join()

            success = True
            message = "Keyboard monitoring finished correctly."

        except Exception as e:
            success = False
            message = e

        return success,message