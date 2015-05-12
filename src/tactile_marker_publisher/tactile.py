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
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the following
#   disclaimer in the documentation and/or other materials provided
#   with the distribution.
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


class Range(object):
	def __init__(self, min = float('inf'), max=float('-inf')):
		self._min = min
		self._max = max

	def update(self, value):
		self._min = min(self._min, value)
		self._max = max(self._max, value)

	def min(self):
		return self._min

	def max(self):
		return self._max

	def get(self):
		return self._min, self._max

	def range(self):
		return self._max - self._min


class TactileValue(object):
	"""
	Provide dynamic calibration (to observed data range) and smoothing of single values
	"""
	rawCurrent = 0
	rawMean    = 1
	absCurrent = 2
	absMean    = 3
	relCurrent = 4
	relMean    = 5
	relCurrentRelease = 6
	relMeanRelease = 7

	def __init__(self, min = float('inf'), max=float('-inf')):
		self.mean_lambda = 0.7
		self.range_lambda = 0.9995
		self.release_lambda = 0.05

		self.abs_range = Range(min=min, max=max) # ever seen min/max values
		self.dyn_range = Range() # dynamically decayed range

		self._current = None
		self._mean = None
		self._released = None

	def update(self, data):
		"""
		update internal variables from new data value
		:type data: numerical
		"""
		self.abs_range.update(data)
		self.dyn_range.update(data)
		self.dyn_range._min = data - self.range_lambda * (data - self.dyn_range.min())
		self.dyn_range._max = data + self.range_lambda * (self.dyn_range.max() - data)

		if self._current is None: # first update: init vars and return
			self._current = data
			self._mean = data
			return

		# update mean
		self._mean = data + self.mean_lambda * (self._mean - data)

		# compute release mode
		margin = 0.1 * self.abs_range.range()
		if self._released is not None and data > self._current + margin:
			# if we recently released (_released != None)
			# and new data increased considerably, we leave release mode
			fReleased = None
		elif self._released is None and data < self._current - margin:
			# if new data drops from old value considerably
			# we enter release mode, remembering the old value in _released
			self._released = self._current
		elif self._released is not None:
			# we are in release mode: linearly decrease _released
			self._released -= self.release_lambda * self.dyn_range.range()
			# leave release mode, if we decayed enough
			if self._released < self.dyn_range.min():
				self._released = None

		self._current = data

	def value(self, mode):
		"""
		retrieve current sensor value with given mode
		:type mode: enum
		"""
		if mode == self.rawCurrent: return self._current
		if mode == self.rawMean:    return self._mean

		if mode == self.absCurrent: return self._current - self.abs_range.min()
		if mode == self.absMean:    return self._mean    - self.abs_range.min()

		r =  max(self.dyn_range.range(), 0.1 * self.abs_range.range())
		if r < sys.float_info.epsilon: return 0; # do not divide by zero

		dyn_min = self.dyn_range.min()
		if mode >= self.relCurrentRelease and self._released is not None:
			return -(self._released - dyn_min) / r

		if mode in [self.relCurrent, self.relCurrentRelease]:
			return (self._current - dyn_min) / r

		if mode in [self.relMean, self.relMeanRelease]:
			return (self._mean - dyn_min) / r

		raise Exception("this should not happen")


class TactileValueArray(object):
	"""
	Provide dynamic calibration and smoothing of array values.
	"""
	def __init__(self, min = float('inf'), max=float('-inf')):
		self._min = min
		self._max = max
		self._values = None

	def update(self, data):
		if self._values is None: # first update init values array
			self._values = [TactileValue(self._min, self._max) for d in data]

		for v, d in zip(self._values, data):
			v.update(d)

	def value(self, mode):
		return [v.value(mode) for v in self._values]
