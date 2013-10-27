#!/usr/bin/python
import json,collections,scipy.integrate,scipy.optimize,math
import numpy as np
BezierSpline=collections.namedtuple('BezierSpline','c l r')
class Memo(dict):
	def __init__(self,f):
		self.f=f
	def __missing__(self,key):
		self[key]=self.f(key)
		return self[key]
class Bezier(object):
	'''utility class for immutable n-dimensional Bezier curves
	Bezier curves are parameterized in terms of a single parameter, t.
	This class adds arclength parameterization.
	'''
	def __init__(self,o):
		'create new Bezier from file'
		if isinstance(o,file):
			o=json.load(o)
		self.splines=[BezierSpline(**{k:np.array(v,dtype='float64') for k,v in p.iteritems()}) for p in o]
		self._speed=np.vectorize(self.speed)
		self.integrals=Memo(lambda i:scipy.integrate.quadrature(self._speed,i,i+1)[0])
	def displacement(self,t):
		if t<0:
			s=self.splines[0]
			return (t-1)**3*(s.c-s.l)-s.l+2*s.c
		if t>=len(self.splines)-1:
			s=self.splines[-1]
			t-=len(self.splines)-1
			return (t+1)**3*(s.r-s.c)-s.r+2*s.c
		i=int(t)
		r=self.splines[i]
		s=self.splines[i+1]
		t-=i
		return (1-t)**3*r.c+3*t*(1-t)**2*r.r+3*t**2*(1-t)*s.l+t**3*s.c
	__call__=displacement
	def prime(self,t):
		if t<0:
			s=self.splines[0]
			return 3*(1-t)**2*(s.c-s.l)
		if t>=len(self.splines)-1:
			s=self.splines[-1]
			t-=len(self.splines)-1
			return 3*(t+1)**2*(s.r-s.c)
		i=int(t)
		r=self.splines[i]
		s=self.splines[i+1]
		t-=i
		return 3*(1-t)**2*(r.r-r.c)+6*(1-t)*t*(s.l-r.r)+3*t**2*(s.c-s.l)
	def prime2(self,t):
		if t<0:
			s=self.splines[0]
			return 6*(t-1)*(s.c-s.l)
		if t>=len(self.splines)-1:
			s=self.splines[-1]
			t-=len(self.splines)-1
			return 6*(t+1)*(s.r-s.c)
		i=int(t)
		r=self.splines[i]
		s=self.splines[i+1]
		t-=i
		return 6*(1-t)*(s.l-2*r.r+r.c)+6*t*(s.c-2*s.l+r.r)
	def distance(self,t):
		return sum(map(self.integrals.__getitem__,xrange(int(t))))+scipy.integrate.quadrature(self._speed,int(t),t)[0]
	def speed(self,t):
		return np.linalg.norm(self.prime(t))
	def accel(self,t):
		return np.linalg.norm(self.prime2(t))
	def speed_prime(self,t):
		return np.dot(self.prime(t),self.prime2(t))/self.speed(t)
	def distance_arc(self,d):
		return scipy.optimize.newton(lambda t:self.distance(t)-d,0,fprime=self.speed,fprime2=self.speed_prime)
	def displacement_arc(self,d):
		return self.displacement(self.distance_arc(d))
	def arclength(self):
		return self.distance(len(self.splines)-1)
if __name__=="__main__":
	import sys
	for x in sys.argv[1:]:
		with open(x) as f:
			b=Bezier(f)
			cameras=[]
			for d in np.linspace(0,b.arclength(),num=40):
				t=b.distance_arc(d)
				d=b(t)
				y=b.prime(t)
				k=np.array([0.,0.,1.])
				z=k-y*np.dot(k,y)/np.dot(y,y)
				x=np.cross(y,z)

				x/=np.linalg.norm(x)
				y/=np.linalg.norm(y)
				z/=np.linalg.norm(z)

				ppt=np.identity(4)
				ppt[3][2]=-math.atan(120*math.pi/180.)#parallax
				cameras.append(np.matrix(ppt)*np.concatenate((np.matrix([x,y,z,d]).transpose(),np.matrix([[0,0,0,1]]))).I)

				print np.around(cameras[-1]*cameras[0].I,decimals=3)
