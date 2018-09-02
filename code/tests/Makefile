PROJECT_DIR= ..
SRC_DIR=$(PROJECT_DIR)
TEST_DIR=$(PROJECT_DIR)/tests


libopenCM3Path = $(PROJECT_DIR)/libopencm3
HARDWARE_INCLUDE_DIR = $(libopenCM3Path)/include

# specify where the source code and includes are located
include $(PROJECT_DIR)/Makefile.config

#INCLUDE_FOLDERS = 
INCLUDE_DIRS=$(foreach d, $(INCLUDE_FOLDERS), -I $(PROJECT_DIR)/$d) -I $(HARDWARE_INCLUDE_DIR)
 
# specify where the test code is located
TEST_SRC_DIRS = $(TEST_DIR)
 
# what to call the test binary
TEST_TARGET=example
 
# where the cpputest library is located
CPPUTEST_HOME = ../../../../cpputest-3.7dev

# Specifiy for cross-compliation
CC = gcc
CXX = g++
LD = g++
GCOV = gcov
BROWSER = explorer
 

CSRCS = $(foreach d, $(HL_LIBS), $(PROJECT_DIR)/$d)
COBJS = $(CSRCS:.c=.o)

TEST_CXXSRCS = $(TEST_DIR)/main.cpp $(TEST_DIR)/debounceTest.cpp
TEST_CXXOBJS = $(TEST_DIR)/main.o $(TEST_DIR)/debounceTest.o

ALL_OBJS = $(TEST_CXXOBJS) $(COBJS)

TARGET = test


CFLAGS    := -g -Wall
CFLAGS  += -std=c99
CFLAGS  += -D CPPUTEST            # Compile the test file, ignore the main function of sample.c, the sample.c code with the CPPUTEST macro
CFLAGS += -DSTM32F1


# CPPUTest is C++Write, so using g++ To compile the test file
CPP     := g++
CPPFLAGS  := -g -Wall
CPPFLAGS  += -I $(CPPUTEST_HOME)/include
CXXFLAGS = $(CPPFLAGS)

LD_LIBRARIES := -L $(CPPUTEST_HOME)/lib -lCppUTest -lCppUTestExt



all: $(TARGET)

objects: $(ALL_OBJS)

# Link them all together/compile the executable
$(TARGET): $(ALL_OBJS)
	$(LD) -o $(TARGET) $(ALL_OBJS) $(LD_LIBRARIES)

	
#$(TEST_CXXOBJS): 
	
# Compilation
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) $(ARCH_FLAGS) -o $(*).o -c $(*).cpp
	
%.o: %.c
	@#printf "  CC      $(*).c\n"
	$(Q) $(CC) $(CFLAGS) $(INCLUDE_DIRS) $(CPPFLAGS) $(ARCH_FLAGS) -o $(*).o -c $(*).c

	
.PHONY: clean
clean:
	@echo "clean..."
	rm -rf tests
	rm $(ALL_OBJS)
	rm $(TARGET).exe

testecho:
	@echo $(TEST_CXXOBJS)
	@echo $(CSRCS)
	@echo $(COBJS)
































