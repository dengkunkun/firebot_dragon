#SDK测试程序

CXX   =  g++
CXXFLAGS = -Wall -Wno-strict-aliasing -Wno-unused-variable

#包含头文件路径
SUBDIR   = $(shell ls ../../src -R | grep /)
SUBDIRS  = $(subst :,/,$(SUBDIR))
INCPATHS = $(subst ../../,-I../../,$(SUBDIRS))
INCPATHS += -I../../include/

VPATH = $(subst : ,:,$(SUBDIR))./
SOURCE = $(foreach dir,$(SUBDIRS),$(wildcard $(dir)*.cpp))

#将cpp的路径名称的.cpp后缀换为.o
OBJS = $(patsubst %.cpp,%.o,$(SOURCE))
#取出.o的文件名，不带路径
OBJFILE  = $(foreach dir,$(OBJS),$(notdir $(dir)))
#.o输出到.OBJ目录下
OBJSPATH = $(addprefix obj/,$(OBJFILE)) 

LIBPATH = ../lib/ -L../MakeAll
LIBS = -Wl,-rpath=./:./HCNetSDKCom:../lib:../MakeAll -lhcnetsdk
EXE = ../lib/sdkTest

$(EXE):$(OBJFILE)
	$(CXX) -L$(LIBPATH)  -o $(EXE) $(OBJFILE) $(INCPATHS) $(LIBS)

$(OBJFILE):%.o:%.cpp
	$(CXX)  -c -o $@ $<  $(INCPATHS) -pipe -g -Wall

#依赖文件
DPPS = $(patsubst %.cpp,%.dpp,$(SOURCE))
include $(DPPS)
%.dpp: %.cpp	
	g++ $(INCPATHS) -MM $(CPPFLAGS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$ 
	

.PHONY:clean
clean:
	rm -rf $(OBJFILE)
	rm -rf $(DPPS)
	rm -rf $(EXE)

