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
			for image_num,d in enumerate(np.linspace(0,b.arclength(),num=40),1):
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
				ppt[3][2]=-math.atan(120*math.pi/360.)#parallax
				ppt=np.matrix(ppt)
				translate=np.concatenate((np.matrix([[0]*3]*3+[d]).T,[[0]*4]))+np.identity(4)
				rotate=np.concatenate((np.matrix([x,y,z,np.zeros(3)]).T,((0,0,0,1),)))
				#cameras.append(ppt*np.concatenate((np.matrix([x,y,z,d]).T,np.matrix([[0,0,0,1]]))).I)
				cameras.append(ppt*translate*rotate)

				fudge=np.matrix(np.diag((1.,-1.,1.,1.0573374023762807)))#different NDC?!?
				trans=fudge.I*cameras[-1]*cameras[0].I*fudge
				print>>sys.stderr,np.around(trans,decimals=2)
				r2d=180/math.pi
				trans2=np.matrix([
					[ 0, 1, 0, 0],
					[-1, 0, 0, 0],
					[ 0, 0, 1, 0],
					[ 0, 0, 0, 1]])*trans
				print '#-hugin  cropFactor=1\ni w256 h256 f0 v%s Ra=0 Rb=0 Rc=0 Rd=0 Re=0 Eev=0 Er%s Eb%s r%f p%f y%f TrX%f TrY%f TrZ%f j0 a=0 b=0 c=0 d=0 e=0 g=0 t=0 Va%s Vb=0 Vc=0 Vd=0 Vx=0 Vy=0  Vm5 n"%04d.png"'.translate(None,'='[image_num-1:])%(
					'120' if image_num==1 else '=0',
					'1' if image_num==1 else '=0',
					'1' if image_num==1 else '=0',
					r2d*math.atan2(trans[0,1],trans[0,0]),#-r2d*math.atan2(trans[0,2],trans[1,2]),#XXX formulas are wrong
					0,#r2d*math.acos(max(-1,min(1,trans[2,2]))),
					0,#r2d*math.atan2(trans[2,0],trans[2,1]),
					trans2[0,3],
					trans2[1,3],
					trans2[2,3],
					'1' if image_num==1 else '=0',
					image_num,
				)
