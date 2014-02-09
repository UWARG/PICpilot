#!/bin/bash
while read count host
do
	end=$[start+count]
	</dev/null ssh "$host" "cd $PWD; seq $[start+1] $end"' | time ionice -c3 nice xargs -n1 time blender -b terrain.blend -x 1 -o //output-4000x3000-jpg/ -F JPEG -f' &
	#</dev/null ssh "$host" killall xargs blender &
	start=$end
done << EOF
13 linux024
13 linux028
14 linux032
0 ubuntu1204-002
0 ubuntu1204-004
0 ubuntu1204-006
EOF
wait
