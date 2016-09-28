PROJECT_NAME=slam

# DIRS
BUILD_DIR = $(PWD)/build
BIN_DIR = $(BUILD_DIR)/bin

LIB_DIR = $(BUILD_DIR)/lib
LIB_BUILD_DIR = $(BUILD_DIR)/$(PROJECT_NAME)

TESTS_BIN_DIR = $(BIN_DIR)/tests
TESTS_BUILD_DIR = $(BUILD_DIR)/tests

UTILS_BIN_DIR = $(BIN_DIR)/util
UTILS_BUILD_DIR = $(BUILD_DIR)/util


# C COMPILER
CXX = g++ -std=c++0x -g
WARN_FLAGS = -Wall -Wextra -Wno-variadic-macros
OPENCV_CFLAGS=`pkg-config --cflags opencv`
OPENCV_LIBS=`pkg-config --libs opencv`
INCLUDES = \
	-I/usr/include \
	-I/usr/local/include \
	-I$(PWD)/include \
	-I/usr/include/eigen3
LIBS = \
	-L/usr/lib \
	-L/usr/local/lib \
	-L$(LIB_DIR) \
	-l$(PROJECT_NAME) \
	-lpthread \
	-ldl \
	$(OPENCV_LIBS) \
	-lginac
CFLAGS = $(WARN_FLAGS) $(INCLUDES) $(OPENCV_CFLAGS)


# ARCHIVER
AR = ar
ARFLAGS = crs


# COMMANDS
MAKE_OBJ = \
	echo "CPP [$<]"; \
	$(CXX) $(CFLAGS) -c $< -o $@;

MAKE_OBJ_SILENT = \
	$(CXX) $(CFLAGS) -c $< -o $@;

MAKE_TEST = \
	echo "TEST [$<]"; \
	$(CXX) -c $< -o $@ $(CFLAGS); \
	$(CXX) $@ -o $(addprefix $(TESTS_BIN_DIR)/, $(notdir $(@:.o=))) $(LIBS);

MAKE_UTIL = \
	echo "UTIL [$<]"; \
	$(CXX) -c $< -o $@ $(CFLAGS); \
	$(CXX) $@ -o $(addprefix $(UTILS_BIN_DIR)/, $(notdir $(@:.o=))) $(LIBS);

MAKE_STATIC_LIB = \
	echo "AR [$@]"; \
	$(AR) $(ARFLAGS) $@ $(wildcard $(LIB_BUILD_DIR)/*.o);
