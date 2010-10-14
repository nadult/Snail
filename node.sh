#!/bin/bash
if [ `uname -m` = "x86_64" ]
then
	cd /workspace1/rtracer && ./node
else
	cd $HOME/rtbin && ./node_ppc
fi
