profile:
	-rm callgrind.out* cachegrind.out*
	valgrind --tool=callgrind ./rtracer 512 512 0 1
	kcachegrind

profilec:
	-rm callgrind.out* cachegrind.out*
	valgrind --tool=cachegrind ./rtracer 512 512 0 1
	kcachegrind

.PHONY: profile profilec
