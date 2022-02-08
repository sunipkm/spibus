.PHONY: doc

doc:
	doxygen .doxyconfig

.PHONY: clean

clean:
	rm -vf *.out
	rm -vf *.o

spotless: clean
	rm -vrf doc