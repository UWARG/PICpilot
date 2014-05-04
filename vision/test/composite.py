#!/usr/bin/env python2
from __future__ import print_function
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
import multiprocessing
try:
	xrange
except NameError:
	xrange=range
def _mean_residual(a):
	m=np.array(a).mean()
	return m,a-m
Composite=collections.namedtuple("Composite","path ready to_gps find_images")
class Map(object):
	types=("nN",int),("xXyYrpy",float)
	_make=r"C:\MinGW\bin\mingw32-make.exe"
	if not os.path.exists(_make):
		_make="make"
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
					#print(line)
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
			image["v"]="90"
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
		idx={i for i in xrange(len(self.images))if size(i,i)>=10} # XXX at least 10 images
		components=[{i for i,e in enumerate(index)if e==j}for j in idx]
		reverse={e:i for i,e in enumerate(x for primary in components for x in primary)}
		self._components=[sorted([reverse[i]for i in primary])for primary in components]
		self.images=[self.images[x]for primary in components for x in primary]
		new_points=[]
		for point in points:
			for field in"nN":
				try:
					point[field]=reverse[point[field]]
				except KeyError:
					break
			else:
				point["line"]="c n%(n)d N%(N)d x%(x)f y%(y)f X%(X)f Y%(Y)f"%point
				new_points.append(point)
		self._points=new_points
	def _output_raw_pto(self,fd):
		"Write a pto for autooptimiser"
		for image in self.images:
			print(image["comment"],file=fd)
			print(image["line"],file=fd)
		for primary in self._components:
			print("v r%d"%primary[0],file=fd)
			for i in primary[1:]: # XXX first image gets way off whack
				print("""\
v r%d
v TrX%d
v TrY%d"""%(i,i,i),file=fd)
		print("v",file=fd)
		for point in self._points:
			print(point["line"],file=fd)
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
			subprocess.check_call(["panomatic","--linearmatch","--linearmatchlen","3","-n",str(multiprocessing.cpu_count()),"-o",pts_pto_path,"--"]+[e["path"]for e in self.images])
			self._read_points(pts_pto_path)

		opt_pto_path=self._root_path+"opt.pto"
		if not os.path.exists(opt_pto_path):
			raw_pto_path=self._root_path+"raw.pto"
			with open(raw_pto_path,"w")as raw_pto_fd:
				self._output_raw_pto(raw_pto_fd)
			subprocess.check_call(("autooptimiser","-n",raw_pto_path,"-o",opt_pto_path))
		self._read_panorama(opt_pto_path)

		self._initialized.set()
	def _read_panorama(self,opt_pto_path):
		"Read optimized panorama template"
		p=hsi.Panorama()
		ifs=hsi.ifstream(opt_pto_path)
		p.readData(ifs)
		del ifs
		po=p.getOptions()
		po.enblendOptions="--primary-seam-generator=graph-cut" # no --gpu
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
		nr=len(self.images)
		self._z=np.zeros(nr,dtype="complex")
		self._u0=np.zeros(nr,dtype="complex")
		self._u=np.zeros(nr,dtype="complex")
		self._g0=np.zeros(nr,dtype="complex")
		self._dgps=np.zeros((nr,2))
		p=self.panorama
		for primary in self._components:#XXX XXX self
			set_primary=set(primary)
			vals=[fields["TrX"].getValue()+1j*fields["TrY"].getValue()for i,fields in enumerate(p.getVariables())if i in set_primary]
			u0,u=_mean_residual(vals[1:])
			u_all=np.concatenate(([vals[0]-u0],u))
			images=[self.images[i]for i in primary]
			g0,g=_mean_residual([image["lat"]+1j*image["lon"]for image in images])
			dlat,dlon=self.meters_per_radian(g0.real)
			(a,b),(r,),_,_=np.linalg.lstsq(
				np.rollaxis(np.array([np.concatenate((u.real,u.imag)),np.concatenate((-u.imag,u.real))]),1),
				np.concatenate((g[1:].real*dlat,g[1:].imag*dlon))
			)
			u=u_all
			#print("gps ~ (%.3g+%.3gj) (x+yj), residual = %.3g"%(a,b,r))
			# found z = (a+bj): g = zu, so find real k, unit w: k(1, 0) = zw(0, -1)
			z=a+1j*b
			g=z*u
			for image,clat,clon in zip(
				images,
				g0.real+g.real/dlat,
				g0.imag+g.imag/dlon
			):
				image["lat"]=clat
				image["lon"]=clon
			self._z[primary]=z
			self._u0[primary]=u0
			self._u[primary]=u_all
			self._g0[primary]=g0
			self._dgps[primary]=dlat,dlon
	def createComposite(self,topLeftLat,topLeftLon,topRightLat,topRightLon,widthPx,heightPx):
		"Starts creating a composite image. Returns composite id immediately."
		topLeftLat*=math.pi/180
		topLeftLon*=math.pi/180
		topRightLat*=math.pi/180
		topRightLon*=math.pi/180
		self._initialized.wait()
		p=self.panorama
		u=self._u
		z=self._z # g = zu
		dlat,dlon=self._dgps.transpose()
		g0=self._g0
		tl=(topLeftLat -g0.real)*dlat+(topLeftLon -g0.imag)*dlon*1j
		tr=(topRightLat-g0.real)*dlat+(topRightLon-g0.imag)*dlon*1j

		z*=1j # conversion factor
		nz=(tr-tl)/2 # new z, (tr-tl)/(2*math.tan(hfov/2))*-1j==(tr-tl)/2*-1j
		#m,phi=cmath.polar(z/nz)
		z_nz=z/nz
		m=np.abs(z_nz)
		phi=np.angle(z_nz)
		mr=phi*(180/math.pi)
		nu=(u*z-tl)/nz # math.tan(hfov/2)==1
		for img,fields,image,m,mr in zip(
			p.getActiveImages(),
			p.getVariables(),
			self.images,
			m,
			mr,
		):
			nu[img]-=1+1j*image["height"]/image["width"]
			for k,v in{
				"TrX":nu[img].real,
				"TrY":nu[img].imag,
				"TrZ":m/2-1,
				"r":(fields["r"].getValue()+mr+180)%360-180,
			}.items():
				fields[k].setValue(v)
				p.updateVariable(img,fields[k])
		def to_gps(py,px):
			g=tl+(tr-tl)*((px+1j*py)/widthPx)
			return(g0.real+g.real/dlat)*(180/math.pi),(g0.imag+g.imag/dlon)*(180/math.pi)
		def find_images(py,px):
			ret=[]
			px=(2.*px-widthPx)/widthPx
			py=(2.*py-heightPx)/widthPx
			for img,fields,image in zip(
				p.getActiveImages(),
				p.getVariables(),
				self.images
			):
				v=m*cmath.exp(1j*fields["r"].getValue()*(math.pi/180))
				#print(img,((px+1j*py)-nu[img])/v,end='')
				c=(((px+1j*py)-nu[img])/v+.5)*image["width"]+.5j*image["height"]
				#print(c)
				if 0<=c.real<image["width"]and 0<=c.imag<image["height"]:
					ret.append((image["path"],c.imag,c.real))
			return ret

		orig=p.getMemento()
		po=p.getOptions()
		po.outputImageType="png"
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
		composite=Composite(
			path=self._base_path+"out.png",
			ready=threading.Event(),
			to_gps=to_gps,
			find_images=find_images,
		)
		self.composites.append(composite)
		def cb():
			subprocess.check_call(("pto2mk",out_pto_path,"-o",out_mk_path,"-p",os.path.join(os.path.dirname(out_mk_path),"out")))
			subprocess.call((self._make,"-j","-f",out_mk_path)) # could fail
			composite.ready.set()
		threading.Thread(target=cb).start()
		return len(self.composites)-1
	def isCompositeReady(self,cid):
		"Returns true if composite is ready."
		return self.composites[cid].ready.is_set()
	def composite2Path(self,cid):
		"Waits until isCompositeReady(id) and returns the path to the image. No transparent images are saved."
		composite=self.composites[cid]
		composite.ready.wait()
		return composite.path
	def composite2GPS(self,cid,compositeRow,compositeCol):
		"Returns the GPS coordinates for the composite given image coordinates, approximating GPS coordinates as being linear."
		return self.composites[cid].to_gps(compositeRow,compositeCol)
	def composite2Images(self,cid,compositeRow,compositeCol):
		"Returns a list of images for a composite at given coordinates."
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
		self._z=None
		self._u=None
		self._u0=None
		self._g0=None
		self._dgps=None
		self._initialized=threading.Event()
		self.composites=[]
		self._worker=threading.Thread(target=self._deferred_init)
		self._worker.start()
def main():
	import sys
	sys.setrecursionlimit(10000)
	path=sys.argv[1]
	m=Map(path)
	cids=[]
	cids.append(m.createComposite(49.911688877,-98.285622596,49.911523060,-98.267898559,1000,1000))
	print("started compositing",cids)
	print("done compositing","\n".join(map(m.composite2Path,cids)))
	#print("images",m.composite2Images(cids[3],50,400))
	#print("GPS",m.composite2GPS(cids[3],50,400))#0,2.4
	#print("images",m.composite2Images(m.createComposite(0,2,0,3,100,70),5,40))
	#print("GPS",m.composite2GPS(m.createComposite(0,2,0,3,100,70),5,40))#0,2.4
if __name__=="__main__":
	main()
# vim: set ts=4 sw=4:
