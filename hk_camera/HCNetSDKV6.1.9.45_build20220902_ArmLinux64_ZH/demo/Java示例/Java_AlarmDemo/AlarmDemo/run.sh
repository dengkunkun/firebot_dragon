#!/bin/bash
cd ./src

#编译   引入需要的依赖的jar以及java文件，以及运行文件
CP+=../lib/jna.jar:
CP+=../lib/examples.jar:
echo "--> ClassPath: $CP"
#SRC_LIB=com/netsdk/lib/*.java 
SRC_EXAMPLE=Test1/*.java 

BIN+=../bin
echo "--> Bin $BIN"

javac -cp $CP $SRC_EXAMPLE -d $BIN

#指定库的路径
export LD_LIBRARY_PATH=../lib/:../lib/HCNetSDKCom/

#运行
cd ../bin
echo "--> path:" pwd

DEMO=Test1/Test1

java -cp $CP:. $DEMO
cd -
