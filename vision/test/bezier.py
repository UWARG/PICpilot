#!/usr/bin/python
import json,collections,scipy.integrate,scipy.optimize,math,scipy
import numpy as np
np.set_printoptions(precision=2)
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
		self.sums=Memo(lambda i:self.sums[i-1]+self.integrals[i-1])
		self.sums[0]=0
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
		return self.sums[max(0,int(t))]+scipy.integrate.quadrature(self._speed,int(t),t)[0]
	def speed(self,t):
		return np.linalg.norm(self.prime(t))
	def accel(self,t):
		return np.linalg.norm(self.prime2(t))
	def speed_prime(self,t):
		return np.dot(self.prime(t),self.prime2(t))/self.speed(t)
	def distance_arc(self,d):
		#return scipy.optimize.newton(lambda t:self.distance(t)-d,0,fprime=self.speed,fprime2=self.speed_prime,maxiter=100)
		start=-1.
		end=1.
		while self.distance(end)<d:
			start=end
			end*=2
		while self.distance(start)>d:
			end=start
			start*=2
		return scipy.optimize.brentq(lambda t:self.distance(t)-d,start,end)
	def displacement_arc(self,d):
		return self.displacement(self.distance_arc(d))
	def arclength(self):
		return self.distance(len(self.splines)-1)
if __name__=="__main__":
	import sys
	import subprocess
	dimensions=subprocess.check_output(("identify","-format","w%w h%h","output/0001.jpg")).strip()
	for x in sys.argv[2:]:
		with open(x) as f:
			b=Bezier(f)
			cameras=[]#transforms half ndc to object space
			for image_num,d in enumerate(np.linspace(0,b.arclength(),num=int(sys.argv[1]),endpoint=False),1):
				t=b.distance_arc(d)
				d=b(t)
				dy=b.prime(t)

				#print>>sys.stderr,"displacement",d
				#print>>sys.stderr,"velocity",dy

				k=np.array([0.,0.,1.])
				dz=k-dy*np.dot(k,dy)/np.dot(dy,dy)
				dx=np.cross(dy,dz)

				dx/=np.linalg.norm(dx)
				dy/=np.linalg.norm(dy)
				dz/=np.linalg.norm(dz)
				#print>>sys.stderr,"dx",dx,"dy",dy,"dz",dz

				wpt_to_obj=np.concatenate((np.matrix([dx,dy,dz,d]).T,[[0,0,0,1]]))
				#print>>sys.stderr,"wpt_to_obj"
				#print>>sys.stderr,wpt_to_obj
				cameras.append(wpt_to_obj*np.matrix(np.diag((-1,1,1,1))))

				trans=cameras[0].I*cameras[-1]
				#print>>sys.stderr,"trans"
				#print>>sys.stderr,trans
				#"""
				r2d=180/math.pi
				print'#-hugin  cropFactor=1\ni %s f0 v%s Ra=0 Rb=0 Rc=0 Rd=0 Re=0 Eev=0 Er%s Eb%s r%f p%f y%f TrX%f TrY%f TrZ%f j0 a=0 b=0 c=0 d=0 e=0 g=0 t=0 Va%s Vb=0 Vc=0 Vd=0 Vx=0 Vy=0  Vm5 n"%04d.jpg"'.translate(None,'='[image_num-1:])%(
					dimensions,
					'120' if image_num==1 else '=0',
					'1' if image_num==1 else '=0',
					'1' if image_num==1 else '=0',
					-r2d*math.atan2(trans[0,1],trans[1,1]),
					-r2d*math.atan2(trans[2,1],trans[2,2]),#XXX fix
					0,#r2d*math.atan2(trans[2,0],trans[2,1]),#XXX fix
					-trans[0,3],
					-trans[1,3],
					-trans[2,3],
					'1' if image_num==1 else '=0',
					image_num,
				)
				#"""
