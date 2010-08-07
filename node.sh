#!/bin/bash
if [ `uname -m` = "x86_64" ]
then
	cd /workspace1/rtracer && LD_PRELOAD=$HOME/gcc-4.5.0/lib64/libstdc++.so:$HOME/gcc-4.5.0/lib64/libgcc_s.so ./node
else
	cd $HOME/rtbin && ./node_ppc
fi
