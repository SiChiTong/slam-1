include $(PWD)/config.mk
SRCS = $(wildcard *.cpp)
OBJS = $(addprefix $(LIB_BUILD_DIR)/, $(SRCS:.cpp=.o))

default: $(LIB_DIR)/lib$(PROJECT_NAME).a

$(LIB_BUILD_DIR)/%.o: %.cpp
	$(MAKE_OBJ)

$(LIB_DIR)/lib$(PROJECT_NAME).a: $(OBJS)
	$(MAKE_STATIC_LIB)
