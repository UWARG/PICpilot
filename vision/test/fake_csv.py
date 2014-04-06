#!/usr/bin/env python2
import bezier
import numpy as np
import sys
import csv
if __name__=="__main__":
	if len(sys.argv)!=4:
		print"usage: %s format num path"%sys.argv[0]
		sys.exit(1)
	fmt=sys.argv[1]
	writer=csv.writer(sys.stdout)
	writer.writerow(("path","lat","lon"))
	with open(sys.argv[3]) as f:
		b=bezier.Bezier(f)
		cameras=[]#transforms half ndc to object space
		for image_num,d in enumerate(np.linspace(0,b.arclength(),num=int(sys.argv[2]),endpoint=False),1):
			x,y,z=b(b.distance_arc(d))
			writer.writerow((fmt%image_num,x,y))
