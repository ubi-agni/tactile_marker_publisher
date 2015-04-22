# Software License Agreement (BSD License)
#
# Copyright (c) 2015, CITEC, Bielefeld University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
from urdf_parser_py import urdf
xmlr = urdf.xmlr # define shortcut

def on_error(message):
	sys.stderr.write(message + '\n')
xmlr.core.on_error = on_error

xmlr.start_namespace('urdf')

class TactileMarker(xmlr.Object):
	""" TactileMarker represents the parsed XML structure of <tactile> tasgs in URDFs """
	def __init__(self, source=None, link=None, geometry=None, origin=None, name=None):
		self.source = source
		self.geometry = geometry
		self.origin = origin
		self.name = name
		self.link = link

xmlr.reflect(TactileMarker, params=[
	xmlr.Attribute('link', str, True),
	xmlr.Attribute('source', str, True),
	xmlr.Element('geometry', 'geometric'),
	urdf.origin_element,
	urdf.name_attribute,
])

class Robot(urdf.Robot):
	def __init__(self):
		super(Robot, self).__init__()
		self.tactiles = []

xmlr.reflect(Robot, tag = 'robot', params = [
# 	name_attribute,
	xmlr.Attribute('name', str, False), # Is 'name' a required attribute?
	xmlr.AggregateElement('link', urdf.Link),
	xmlr.AggregateElement('joint', urdf.Joint),
	xmlr.AggregateElement('gazebo', xmlr.RawType()),
 	xmlr.AggregateElement('transmission', 'transmission'),
	xmlr.AggregateElement('material', urdf.Material),
	xmlr.AggregateElement('tactile', TactileMarker)
	])

# make an alias
URDF = Robot

xmlr.end_namespace()
