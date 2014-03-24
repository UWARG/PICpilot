import sys
import _heapq
sys.modules["_heapq"]=None
import heapq
from heapq import *
class InvList(list):
	def __init__(self,*args,**kwargs):
		list.__init__(self,*args,**kwargs)
		self._rehash()
	def _rehash(self):
		self.inv={}
		for i,e in enumerate(self):
			if e not in self.inv:
				self.inv[e]=set()
			self.inv[e].add(i)
	def __setitem__(self,key,value):
		if key<0:
			key+=len(self)
		old=self[key]
		self.inv[old].remove(key)
		if not self.inv[old]:
			del self.inv[old]
		list.__setitem__(self,key,value)
		if value not in self.inv:
			self.inv[value]=set()
		self.inv[value].add(key)
	def __contains__(self,value):
		return value in self.inv
	def __delslice__(self,start,stop):
		if start<0:
			start+=len(self)
		if stop<0:
			stop+=len(self)
		for i in xrange(start,len(self)):
			e=self[i]
			self.inv[e].remove(i)
			if not self.inv[e]:
				del self.inv[e]
		list.__delslice__(self,start,stop)
		for i in xrange(start,len(self)):
			e=self[i]
			if e not in self.inv:
				self.inv[e]=set()
			self.inv[e].add(i)
	def __delitem__(self,index):
		if index<0:
			index+=len(self)
		del self[index:index+1]
	def append(self,value):
		list.append(self,value)
		if value not in self.inv:
			self.inv[value]=set()
		self.inv[value].add(len(self)-1)
	def count(self,value):
		return len(self.inv[value])if value in self.inv else 0
	def extend(self,iterable):
		for value in iterable:
			self.append(value)
	def index(self,value,start=0,stop=float("inf")):
		if start<0:
			start+=len(self)
		if stop<0:
			stop+=len(self)
		if value in self.inv:
			try:
				return min(i for i in self.inv[value]if start<=i<stop)
			except ValueError:
				pass
		list.index(value,0,0)
	def insert(self,index,value):
		if index<0:
			index+=len(self)
		index=max(0,min(len(self),index))
		self.append(self[-1])
		for i in xrange(index,len(self)-1):
			self[i+1]=self[i]
		self[index]=value
	def pop(self,index=-1):
		ret=self[index]
		del self[index]
		return ret
	def remove(self,value):
		del self[self.index(value)]
	def reverse(self,*args,**kwargs):
		list.reverse(self,*args,**kwargs)
		self._rehash()
	def sort(self,*args,**kwargs):
		list.sort(self,*args,**kwargs)
		self._rehash()
def heapify(heap):
	_heapq.heapify(heap)
	if isinstance(heap,InvList):
		heap._rehash()
def heapchange(heap,old,new):
	i=heap.index(old)
	heap[i]=new
	heapq._siftup(heap,i)
def _check(heap):
	inv=heap.inv
	heap._rehash()
	assert inv==heap.inv
if __name__=="__main__":
	import random
	q=InvList(xrange(10))
	_check(q)
	random.shuffle(q)
	_check(q)
	heapify(q)
	_check(q)
	for i in xrange(20,30):
		heappush(q,i)
		_check(q)
	for i in xrange(20,30):
		heapchange(q,i,i-10)
		_check(q)
	prev=0
	while q:
		cur=heappop(q)
		_check(q)
		assert prev<=cur
		prev=cur
