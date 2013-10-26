output: terrain.blend map.png
	blender -b terrain.blend -x 1 -o //output/ -F jpeg -a 
map.png:
	convert -size 300x300 plasma:fractal -blur 0x5 -emboss 1 +level-colors '#0f0,#a83' map.png
clean:
	rm -rf output map.png
.PHONY: clean
