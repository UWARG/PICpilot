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
import os.path
import datetime
def _mean_residual(a):
	m=np.array(a).mean()
	return m,a-m
class Map(object):
	types=("nN",int),("xXyYrpy",float)
	def _read_points(self,pts_pto_path):
		"Read image paths and control points"
		with open(pts_pto_path)as points_fd:
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
		try:
			os.mkdir(self._root_path)
		except OSError:
			pass
		with open(self._csv_path)as csv_fd:
			reader=csv.reader(csv_fd)
			try:
				index={e:i for i,e in enumerate(next(reader))}
			except StopIteration:
				raise EOFError
			self.images=[]
			for row in reader:
				self.images.append({
					"path":os.path.abspath(row[index["path"]]),
					"lat":float(row[index["lat"]])*(math.pi/180),
					"lon":float(row[index["lon"]])*(math.pi/180),
					"width":None,
					"height":None,
				})

		pts_pto_path=self._root_path+"pts.pto"
		try:
			self._read_points(pts_pto_path)
		except IOError:
			subprocess.check_call(["panomatic","-o",pts_pto_path,"--"]+[e["path"]for e in self.images])
			self._read_points(pts_pto_path)

		opt_pto_path=self._root_path+"opt.pto"
		if not os.path.exists(opt_pto_path):
			raw_pto_path=self._root_path+"raw.pto"
			with open(raw_pto_path,"w")as raw_pto_fd:
				self._output_raw_pto(raw_pto_fd)
			subprocess.check_call(("autooptimiser","-n",raw_pto_path,"-o",opt_pto_path))
		self._read_panorama(opt_pto_path)

		print self.createComposite(0.,0.,1.,1.,100,100)#XXX
	def _read_panorama(self,opt_pto_path):
		"Read optimized panorama template"
		p=hsi.Panorama()
		ifs=hsi.ifstream(opt_pto_path)
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

		orig=p.getMemento()
		po=p.getOptions()
		po.setWidth(widthPx)
		po.setHeight(heightPx)
		po.setROI(hsi.Rect2D(0,0,widthPx,heightPx))
		del po
		while True:
			self._base_path=self._root_path+datetime.datetime.utcnow().isoformat().replace(":","_")+os.path.sep
			try:
				os.mkdir(self._base_path)
				break
			except OSError:
				pass
		out_pto_path=self._base_path+"out.pto"
		ofs=hsi.ofstream(out_pto_path)
		p.writeData(ofs)
		del ofs
		p.setMemento(orig)
		out_mk_path=out_pto_path+".mk"
		subprocess.check_call(("pto2mk",out_pto_path,"-o",out_mk_path,"-p",os.path.join(os.path.dirname(out_mk_path),"out")))
		subprocess.check_call(("make","-f",out_mk_path))
		return self._base_path+"out.tif"
	def isCompositeReady(self,cid):
		"Returns true if composite is ready."
		raise NotImplementedError
		return self.composites[cid].ready
	def composite2Path(self,cid):
		"Waits until isCompositeReady(id) and returns the path to the image."
		raise NotImplementedError
		return self.composites[cid].path
	def composite2GPS(self,cid,compositeRow,compositeCol):
		"Returns the GPS coordinates for the composite given image coordinates, approximating GPS coordinates as being linear."
		raise NotImplementedError
		return self.composites[cid].toGPS(compositeRow,compositeCol)
	def composite2Images(self,cid,compositeRow,compositeCol):
		"Returns a list of images for a composite at given coordinates."
		raise NotImplementedError
		return self.composites[cid].find_images(compositeRow,compositeCol)
	def __init__(self,csv_path):
		if not csv_path.endswith(".csv"):
			raise ValueError("invalid csv path")
		self._points=None
		self.images=None
		self.panorama=None
		self._csv_path=csv_path
		self._root_path=csv_path[:-4]+os.path.sep
		self._base_path=None
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
	m=Map("test.csv")
if __name__=="__main__":
	main()
# vim: set ts=4 sw=4:
