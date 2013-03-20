CC=ccache gcc -w -g -O3
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

OBJS=BowVector.o FSurf64.o FSurf128.o FBrief.o FNarf.o ScoringObject.o QueryResults.o FeatureVector.o \
	TwoWayMatcher.o registrorgb.o registro3d.o 

DEPS_SO=DUtils/libDUtils.so DUtilsCV/libDUtilsCV.so DVision/libDVision.so
TARGET=libDBoW2.so

all: $(TARGET) democ demo

demo: $(OBJS) demo.o $(TARGET)
	$(CC) $^ $(LFLAGS) -O3 -o $@

demo.o: demo.cpp $(DEPS)
	$(CC) $(CFLAGS) -O3 -Wall -c $< -o $@

democ: $(OBJS) democ.o $(TARGET)
	$(CC) $^ $(LFLAGS) -O3 -o $@

democ.o: democ.cpp $(DEPS)
	$(CC) $(CFLAGS) -O3 -Wall -c $< -o $@

%.o: %.cpp $(DEPS)
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
	rm -f *.o $(TARGET) demo demonarf voc query && \
	make -C DUtils clean && \
	make -C DUtilsCV clean && \
	make -C DVision clean && \
	rm -f small_db.yml.gz small_voc.yml.gz

install: $(TARGET)
	cp $(TARGET)  /usr/local/lib/ && \
	mkdir -p /usr/local/include/DBoW2 && \
	cp $(DEPS) /usr/local/include/DBoW2 && \
	make -C DUtils install && \
	make -C DUtilsCV install && \
	make -C DVision install

