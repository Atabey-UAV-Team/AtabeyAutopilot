#!/bin/bash

export FG_ROOT=/home/erayacikgz/.fgfs//data
export PATH=$PATH:$FG_ROOT/../bin
export FG_SCENERY=$FG_ROOT/Scenery:$FG_ROOT/WorldScenery

fgfs --fdm=null --native-fdm=socket,in,30,localhost,5502,udp   --prop:/sim/rendering/shaders/quality-level=0 --aircraft=HL20 --fog-fastest --disable-clouds --start-date-lat=2004:06:01:09:00:00 --disable-sound --in-air --airport=KSFO --runway=10L --altitude=7224 --heading=113 --offset-distance=4.72 --offset-azimuth=0   &
