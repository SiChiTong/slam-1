include $(PWD)/config.mk
SRCS = $(wildcard *.cpp)
OBJS = $(addprefix $(TESTS_BUILD_DIR)/, $(SRCS:.cpp=.o))

default: $(OBJS)

$(TESTS_BUILD_DIR)/%.o: %.cpp
	$(MAKE_TEST)
