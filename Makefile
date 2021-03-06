CC=ccache gcc -w -g -O3
CCDBG=ccache gcc -w -g -O3
CFLAGS=-IDUtils -IDUtilsCV -IDVision -DNDEBUG -Wall -O3 -pipe -fopenmp
CFLAGS+=$(shell pkg-config --cflags opencv )
#Giacomo Picchiarelli
CFLAGS+= -I /usr/include/vtk-5.8 -I /usr/include/pcl-1.6 -I /usr/include/eigen3 -Iinclude -Isure3d/include 
#CFLAGS+= -I /usr/local/include/pcl-1.7
######
LFLAGS= -lopencv_core -LDUtils -LDUtilsCV -LDVision $(shell pkg-config --libs opencv) \
 -lpcl_apps -lDVision -lDUtilsCV -lDUtils -lstdc++ \
 -lpcl_common -lpcl_features -lpcl_filters -lpcl_io -lpcl_kdtree -lpcl_keypoints -lpcl_octree \
-lpcl_registration -lpcl_sample_consensus \
-lpcl_search -lpcl_segmentation -lpcl_surface -lpcl_visualization \
-lvtkCommon -lvtkFiltering -lvtkRendering -lpthread -lboost_thread -lboost_system -lboost_iostreams\
-lboost_serialization -lsure3d -lvtkRendering -lvtkGraphics -lvtkIO -lvtkFiltering -lvtkCommon -lvtksys -lvtkImaging

# con la 1.6 -lpcl_apps

DEPS=BowVector.h FClass.h FSurf64.h FSurf128.h FBrief.h FNarf.h FSure.h ScoringObject.h TemplatedVocabulary.h \
  TemplatedDatabase.h QueryResults.h  FeatureVector.h DBoW2.h TwoWayMatcher.h registrorgb.h registro3d.h lk.h\
stats.h

OBJS=build/BowVector.o build/FSurf64.o build/FSure.o build/FSurf128.o build/FBrief.o build/FNarf.o build/ScoringObject.o \
build/QueryResults.o build/FeatureVector.o build/TwoWayMatcher.o build/registrorgb.o build/registro3d.o build/stats.o \

DEPS_SO=DUtils/libDUtils.so DUtilsCV/libDUtilsCV.so DVision/libDVision.so build/lib/libyaml-cpp.so sure3d/lib/libsure3d.so
TARGET=build/lib/libDBoW2.so

all: $(TARGET) build/lk build/surelp

build/surelp: $(OBJS) build/surelp.o
	$(CC) $(CFLAGS) $^ $(LFLAGS) -O3 -o $@

build/surelp.o: surelp.cpp $(DEPS)
	$(CC) $(CFLAGS) -fPIC -O3 -Wall -c $< -o $@

build/lk: $(OBJS) build/lk.o
	$(CC) $(CFLAGS) $^ $(LFLAGS) -O3 -o $@

build/lk.o: lk.cpp $(DEPS)
	$(CC) $(CFLAGS) -fPIC -O3 -Wall -c $< -o $@

build/%.o: %.cpp $(DEPS)
	$(CC) $(CFLAGS) -fPIC -O3 -Wall -c $< -o $@

$(TARGET): $(OBJS) $(DEPS_SO)
	$(CC) $(OBJS) $(LFLAGS) -shared -o $@

DUtils/libDUtils.so:
	make -C DUtils

DUtilsCV/libDUtilsCV.so:
	make -C DUtilsCV

DVision/libDVision.so:
	make -C DVision

clean:
	rm -f *.o $(TARGET) build/lk pcd_tb && \
	rm -f build/*.o $(TARGET) lk && \
	$(MAKE) -C DUtils clean && \
	$(MAKE) -C DUtilsCV clean && \
	$(MAKE) -C DVision clean
	

install: $(TARGET)
	cp $(TARGET)  /usr/local/lib/ && \
	mkdir -p /usr/local/include/DBoW2 && \
	cp $(DEPS) /usr/local/include/DBoW2 && \
	$(MAKE) -C DUtils install && \
	$(MAKE) -C DUtilsCV install && \
	$(MAKE) -C DVision install

