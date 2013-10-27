test.pto: test.sh output bezier.py path.json
	./test.sh > $@
output: terrain.blend map.png
	blender -b terrain.blend -x 1 -o //output/ -F png -a 
map.png:
	convert -size 300x300 plasma:fractal -blur 0x5 -emboss 1 +level-colors '#0f0,#a83' map.png
path.json: terrain.blend export_path.py
	blender -b terrain.blend -P export_path.py 2> $@
clean:
	rm -rf output map.png
.PHONY: clean
