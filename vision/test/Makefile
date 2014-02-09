show: output/output_hdr.exr
	display output/output_hdr.exr
output/raw_output.pto: make-pto.sh output bezier.py path.json
	./make-pto.sh path.json > $@
output/output.pto: output/raw_output.pto
	autooptimiser -n $< -o $@
output/output.pto.mk: output/output.pto
	cd output && pto2mk ../$< -o ../$@ -p output
output/output_hdr.exr: output/output.pto.mk
	$(MAKE) -C output -f output.pto.mk
output: terrain.blend map.png
	blender -b terrain.blend -x 1 -o //output/ -F JPEG -a
map.png:
	convert -size 300x300 plasma:fractal -blur 0x5 -emboss 1 +level-colors '#0f0,#a83' map.png
path.json: terrain.blend export_path.py
	blender -b terrain.blend -P export_path.py 2> $@
clean:
	rm -rf output map.png
.PHONY: clean show
