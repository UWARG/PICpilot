import bpy,json,sys
for obj in bpy.data.objects:
	if obj.name == "FlightPath" and obj.type=="CURVE" and obj.data==bpy.data.curves['BezierCurve']:
		for spline in obj.data.splines:
			json.dump([{
				'c':point.co,
				'l':point.handle_left,
				'r':point.handle_right,
			} for point in spline.bezier_points], sys.stderr, separators=',:', default=list)
