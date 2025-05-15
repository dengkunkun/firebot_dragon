FLAG = -std=c++11 -c

LFLAG = 

LIB = -L/usr/lib/gcc/x86_64-linux-gnu/12/ -lstdc++ -lusb-1.0
# LIB += -lgcc_s -lgcc -lc

# OUTO = /usr/lib/gcc/x86_64-linux-gnu/12/../../../x86_64-linux-gnu/Scrt1.o
# OUTO += /usr/lib/gcc/x86_64-linux-gnu/12/../../../x86_64-linux-gnu/crti.o
# OUTO += /usr/lib/gcc/x86_64-linux-gnu/12/crtbeginS.o
# OUTO += /usr/lib/gcc/x86_64-linux-gnu/12/crtendS.o
# OUTO += /usr/lib/gcc/x86_64-linux-gnu/12/../../../x86_64-linux-gnu/crtn.o

bt.exe:a.o crc16_modbus.o doraemon_pack.o
	g++ $(LFLAG) crc16_modbus.o doraemon_pack.o a.o $(OUTO) $(LIB)

a.o:usb.cpp
	g++ $(FLAG) -o a.o $^
crc16_modbus.o:crc16_modbus.c
	g++ $(FLAG) -o crc16_modbus.o $^
doraemon_pack.o:doraemon_pack.c
	g++ $(FLAG) -o doraemon_pack.o $^

.PHONY:
clean:
	rm *.o

.PHONY:
db:
	compiledb -n -o ./compile_commands.json make


