#!/bin/sh
cat << EOF
# hugin project file
#hugin_ptoversion 2
p f0 w1878 h3375 v171  E0 R1 n"TIFF_m c:LZW r:CROP"
m g1 i0 f0 m2 p0.00784314

# image lines
EOF
./bezier.py "$@"
cat << EOF

# specify variables that should be optimized
v

#hugin_optimizeReferenceImage 0
#hugin_blender enblend
#hugin_remapper nona
#hugin_enblendOptions 
#hugin_enfuseOptions 
#hugin_hdrmergeOptions -m avg -c
#hugin_outputLDRBlended false
#hugin_outputLDRLayers false
#hugin_outputLDRExposureRemapped false
#hugin_outputLDRExposureLayers false
#hugin_outputLDRExposureBlended false
#hugin_outputLDRStacks false
#hugin_outputLDRExposureLayersFused false
#hugin_outputHDRBlended true
#hugin_outputHDRLayers false
#hugin_outputHDRStacks false
#hugin_outputLayersCompression LZW
#hugin_outputImageType png
#hugin_outputImageTypeCompression LZW
#hugin_outputJPEGQuality 90
#hugin_outputImageTypeHDR exr
#hugin_outputImageTypeHDRCompression LZW
EOF
