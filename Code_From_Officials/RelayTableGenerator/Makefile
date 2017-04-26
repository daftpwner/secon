
default: relayTable.h

combinations: combinations.c
	gcc combinations.c -o combinations

relayTable.h: combinations
	./combinations > relayTable.h

install: relayTable.h
	cp relayTable.h ../ArenaControl

clean: 
	rm -f combinations
