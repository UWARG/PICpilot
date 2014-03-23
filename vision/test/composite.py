#!/usr/bin/env python2
import csv
import subprocess
import threading
import collections
import itertools
import hsi
import math
import cmath
import numpy as np
def _mean_residual(a):
	m=np.array(a).mean()
	return m,a-m
class Map(object):
	types=("nN",int),("xXyYrpy",float)
	def _read_points(self):
		"Read image paths and control points"
		with open(self._points_path)as points_fd:
			prev_img={}
			points=[]
			image=0
			for line in points_fd:
				line=line.strip()
				if not line:
					continue
				if line[0]!='#':
					#print line
					vals={"line":line}
					line=line.split()
					for word in line[1:]:
						for fields,ctor in self.types:
							if word[0]in fields:
								vals[word[0]]=ctor(word[1:])
								break
					if line[0]=='o':
						vals.update(prev_img)
						self.images[image].update(vals)
						image+=1
					elif line[0]=='c':
						points.append(vals)
				elif line.startswith("#-imgfile "):
					width,height,path=line.split()[1:]
					prev_img={
						"comment":line,
						"width":int(width),
						"height":int(height),
						"path":path[1:-1],
					}
		for img,image in enumerate(self.images):
			image["v"]="50"
		self._retain_primary_component(points)
	def _retain_primary_component(self,points):
		"Retains largest connected component of images"
		index=[-1]*len(self.images)
		adj=[set()for _ in index]
		for point in points:
			adj[point['n']].add(point['N'])
			adj[point['N']].add(point['n'])
		def size(idx,cur):
			if index[cur]!=-1:
				return 0
			index[cur]=idx
			return 1+sum(size(idx,neigh)for neigh in adj[cur])
		idx=max((size(i,i),i)for i in xrange(len(self.images)))[1]
		primary=[i for i,e in enumerate(index)if e==idx]
		reverse={e:i for i,e in enumerate(primary)}
		self.images=map(self.images.__getitem__,primary)
		for point in points:
			for field in"nN":
				point[field]=reverse[point[field]]
			point["line"]="c n%(n)d N%(N)d x%(x)f y%(y)f X%(X)f Y%(Y)f"%point
		self._points=points
	def _output_raw_pto(self,fd):
		"Write a pto for autooptimiser"
		for image in self.images:
			print>>fd,image["comment"]
			print>>fd,image["line"]
		for i in xrange(1,len(self.images)):
			print>>fd,"""\
v r%d
v TrX%d
v TrY%d"""%(i,i,i)
		print>>fd,"v"
		for point in self._points:
			print>>fd,point["line"]
	def _deferred_init(self):
		"Worker thread"
		with open(self._csv_path)as csv_fd:
			reader=csv.reader(csv_fd)
			try:
				index={e:i for i,e in enumerate(next(reader))}
			except StopIteration:
				raise EOFError
			self.images=[]
			for row in reader:
				self.images.append({
					"path":row[index["path"]],
					"lat":float(row[index["lat"]])*(math.pi/180),
					"lon":float(row[index["lon"]])*(math.pi/180),
					"width":None,
					"height":None,
				})
		try:
			self._read_points()
		except IOError:
			subprocess.check_call(["panomatic","-o",self._points_path,"--"]+[e["path"]for e in self.images])
			self._read_points()
		try:
			self._read_panorama()
		except IOError:
			with open(self._raw_pto_path,"w")as raw_pto_fd:
				self._output_raw_pto(raw_pto_fd)
			subprocess.check_call(("autooptimiser","-n",self._raw_pto_path,"-o",self._opt_pto_path))
			self._read_panorama()
		m.createComposite(1.,0.,1.,1.,100,100)#XXX
	def _read_panorama(self):
		"Read optimized panorama template"
		p=hsi.Panorama()
		ifs=hsi.ifstream(self._opt_pto_path)
		p.readData(ifs)
		del ifs
		po=p.getOptions()
		po.setProjection(hsi.PanoramaOptions.RECTILINEAR)
		po.setHFOV(90)
		del po
		for img,fields in zip(p.getActiveImages(),p.getVariables()):
			fields["v"].setValue(90)
			p.updateVariable(img,fields["v"])
		self.panorama=p
		self._estimate_gps()
	@staticmethod
	def meters_per_radian(lat):
		"Returns (m west, m north) per radian (anisotropy correction)"
		a=6378137.0 # equatorial radius in m
		f=1/298.257223563 # flattening 1-b/a
		alpha=1-f*(2-f)*math.sin(lat)**2 # non-negative
		a/=math.sqrt(alpha)
		return (1-f)**2*a/alpha,math.cos(lat)*a
	def _estimate_gps(self):
		"Estimate lat/lon values using regression"
		p=self.panorama
		self._u0,self._u=u0,u=_mean_residual([fields["TrX"].getValue()+1j*fields["TrY"].getValue()for fields in p.getVariables()])
		g0,g=_mean_residual([image["lat"]+1j*image["lon"]for image in self.images])
		self._g0=g0
		self._dgps=dlat,dlon=self.meters_per_radian(g0.real)
		(a,b),(r,),_,_=np.linalg.lstsq(
			np.rollaxis(np.array([np.concatenate((u.real,u.imag)),np.concatenate((-u.imag,u.real))]),1),
			np.concatenate((g.real*dlat,g.imag*dlon))
		)
		#print"gps ~ (%.3g+%.3gj) (x+yj), residual = %.3g"%(a,b,r)
		# found z = (a+bj): g = zu, so find real k, unit w: k(1, 0) = zw(0, -1)
		self._z=z=a+1j*b
		g=z*u
		for image,clat,clon in zip(
			self.images,
			g0.real+g.real/dlat,
			g0.imag+g.imag/dlon
		):
			image["lat"]=clat
			image["lon"]=clon
	def createComposite(self,topLeftLat,topLeftLon,topRightLat,topRightLon,widthPx,heightPx):
		"Starts creating a composite image. Returns composite id immediately."
		p=self.panorama
		u=self._u
		z=self._z # g = zu
		dlat,dlon=self._dgps
		g0=self._g0
		tl=((topLeftLat -g0.real)*dlat+(topLeftLon -g0.imag)*dlon*1j)*(math.pi/180)
		tr=((topRightLat-g0.real)*dlat+(topRightLon-g0.imag)*dlon*1j)*(math.pi/180)

		z*=1j # conversion factor
		nz=(tr-tl)/2 # new z, (tr-tl)/(2*math.tan(hfov/2))*-1j==(tr-tl)/2*-1j
		m,phi=cmath.polar(z/nz)
		mr=phi*(180/math.pi)
		nu=((u+self._u0)*z-tl)/nz-(1+1j*heightPx/widthPx) # (1+1j*heightPx/widthPx)*math.tan(hfov/2)==(1+1j*heightPx/widthPx)
		for img,fields,image,cx,cy in zip(
			p.getActiveImages(),
			p.getVariables(),
			self.images,
			nu.real,
			nu.imag,
		):
			for k,v in{
				"TrX":cx,
				"TrY":cy,
				"TrZ":m/2-1,
				"r":(fields["r"].getValue()+mr+180)%360-180,
			}.iteritems():
				fields[k].setValue(v)
				p.updateVariable(img,fields[k])

		path="test_out.pto"#XXX
		#raise NotImplementedError()
		orig=p.getMemento()
		po=p.getOptions()
		po.setWidth(widthPx)
		po.setHeight(heightPx)
		po.setROI(hsi.Rect2D(0,0,widthPx,heightPx))
		del po
		ofs=hsi.ofstream(path)
		p.writeData(ofs)
		del ofs
		p.setMemento(orig)
	def isCompositeReady(self,cid):
		"Returns true if composite is ready."
		raise NotImplementedError
	def composite2Path(self,cid):
		"Waits until isCompositeReady(id) and returns the path to the image."
		raise NotImplementedError
	def composite2GPS(self,cid,compositeRow,compositeCol):
		"Returns the GPS coordinates for the composite given image coordinates, approximating GPS coordinates as being linear."
		raise NotImplementedError
	def composite2Images(self,cid,compositeRow,compositeCol):
		"Returns a list of images for a composite at given coordinates."
		raise NotImplementedError
	def __init__(self,base_path):
		self._points=None
		self.images=None
		self.panorama=None
		self._csv_path=base_path+".csv"
		self._points_path=base_path+"_pts.pto"
		self._raw_pto_path=base_path+"_raw.pto"
		self._opt_pto_path=base_path+"_opt.pto"
		self._worker=threading.Thread(target=self._deferred_init)
		self._worker.isDaemon=False
		self._worker.start()
		self._z=None
		self._u=None
		self._u0=None
		self._g0=None
		self._dgps=None
def main():
	import sys
	path=sys.argv[1]
	with open(path+".csv")as csv_fd:
		m=Map(csv_fd=csv_fd,points_path=path+".oto")
if __name__=="__main__":
	#main()
	m=Map("test")
# vim: set ts=4 sw=4:
