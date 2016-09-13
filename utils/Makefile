include $(PWD)/config.mk
SRCS = $(wildcard *.cpp)
OBJS = $(addprefix $(UTILS_BUILD_DIR)/, $(SRCS:.cpp=.o))

default: $(OBJS)

$(UTILS_BUILD_DIR)/%.o: %.cpp
	$(MAKE_UTIL)
