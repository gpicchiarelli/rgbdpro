CC=ccache gcc -w -O3
CCDBG=ccache gcc -w -g -O3
CFLAGS=-IDUtils -IDUtilsCV -IDVision -DNDEBUG
CFLAGS+=$(shell pkg-config --cflags opencv )
#Giacomo Picchiarelli
CFLAGS+= -I /usr/include/vtk-5.8 -I /usr/include/pcl-1.6 -I /usr/include/eigen3
#####
LFLAGS= -lopencv_core -LDUtils -LDUtilsCV -LDVision $(shell pkg-config --libs opencv) \
  -lDVision -lDUtilsCV -lDUtils -lstdc++ \
-lpcl_apps -lpcl_common -lpcl_features -lpcl_filters -lpcl_io -lpcl_kdtree -lpcl_keypoints -lpcl_octree \
-lpcl_registration -lpcl_sample_consensus \
-lpcl_search -lpcl_segmentation -lpcl_surface -lpcl_visualization \
-lvtkCommon -lvtkFiltering -lvtkRendering -lpthread -lboost_thread

DEPS=BowVector.h FClass.h FSurf64.h FSurf128.h FBrief.h FNarf.h ScoringObject.h TemplatedVocabulary.h \
  TemplatedDatabase.h QueryResults.h  FeatureVector.h DBoW2.h TwoWayMatcher.h registrorgb.h registro3d.h 

OBJS=build/BowVector.o build/FSurf64.o build/FSurf128.o build/FBrief.o build/FNarf.o build/ScoringObject.o build/QueryResults.o build/FeatureVector.o build/TwoWayMatcher.o build/registrorgb.o build/registro3d.o 

DEPS_SO=DUtils/libDUtils.so DUtilsCV/libDUtilsCV.so DVision/libDVision.so
TARGET=build/libDBoW2.so

all: $(TARGET) build/democ

build/democ: $(OBJS) build/democ.o $(TARGET)
	$(CC) $^ $(LFLAGS) -O3 -o $@
	rm -f build/*.o
	make -C DUtils clean && \
	make -C DUtilsCV clean && \
	make -C DVision clean

build/democ.o: democ.cpp $(DEPS)
	$(CC) $(CFLAGS) -O3 -Wall -c $< -o $@

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
	rm -f *.o $(TARGET) build/democ pcd_tb && \
	rm -f build/*.o $(TARGET) democ && \
	make -C DUtils clean && \
	make -C DUtilsCV clean && \
	make -C DVision clean
	

install: $(TARGET)
	cp $(TARGET)  /usr/local/lib/ && \
	mkdir -p /usr/local/include/DBoW2 && \
	cp $(DEPS) /usr/local/include/DBoW2 && \
	make -C DUtils install && \
	make -C DUtilsCV install && \
	make -C DVision install

