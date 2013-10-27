#!/bin/sh
cat << EOF
# hugin project file
#hugin_ptoversion 2
p f0 w256 h114 v176  E0 R1 S0,256,0,448 n"TIFF_m c:LZW r:CROP"
m g1 i0 f0 m2 p0.00784314

# image lines
EOF
./bezier.py path.json | sed 's,n",&output/,'
cat << EOF

# specify variables that should be optimized
v

#hugin_optimizeReferenceImage 0
#hugin_blender enblend
#hugin_remapper nona
#hugin_enblendOptions 
#hugin_enfuseOptions 
#hugin_hdrmergeOptions -m avg -c
#hugin_outputLDRBlended true
#hugin_outputLDRLayers false
#hugin_outputLDRExposureRemapped false
#hugin_outputLDRExposureLayers false
#hugin_outputLDRExposureBlended false
#hugin_outputLDRStacks false
#hugin_outputLDRExposureLayersFused false
#hugin_outputHDRBlended false
#hugin_outputHDRLayers true
#hugin_outputHDRStacks false
#hugin_outputLayersCompression LZW
#hugin_outputImageType tif
#hugin_outputImageTypeCompression LZW
#hugin_outputJPEGQuality 90
#hugin_outputImageTypeHDR exr
#hugin_outputImageTypeHDRCompression LZW
EOF
