CXX := g++

PYTHON_SITE := $(shell python3 -c "import site; print(site.getsitepackages()[0])")
NATIVE := $(PYTHON_SITE)/native
#NATIVE := $(VIRTUAL_ENV)/lib/python3.11/site-packages/native
WPI := $(NATIVE)/ntcore

CXXFLAGS := -std=c++20 -O2 \
    -I$(NATIVE)/ntcore/include \
    -I$(NATIVE)/wpiutil/include \
    -I$(NATIVE)/wpinet/include \
    -I$(NATIVE)/wpimath/include \
    -I$(NATIVE)/wpilib/include \
    -I$(NATIVE)/wpihal/include

LDFLAGS := \
    -L$(NATIVE)/ntcore/lib \
    -L$(NATIVE)/wpinet/lib \
    -L$(NATIVE)/wpiutil/lib \
    -Wl,-rpath,$(NATIVE)/ntcore/lib \
    -Wl,-rpath,$(NATIVE)/wpinet/lib \
    -Wl,-rpath,$(NATIVE)/wpiutil/lib

LDLIBS := -lntcore -lwpinet -lwpiutil
UNAME_S := $(shell uname -s)
ifneq ($(findstring Darwin,$(UNAME_S)),)
	CXXFLAGS += -I/usr/local/include
	LDFLAGS += -L/usr/local/lib
	LDLIBS += -lglfw3 -framework Cocoa -framework OpenGL -framework IOKit -framework CoreVideo -framework QuartzCore
else
	LDFLAGS += -Wl,--disable-new-dtags
	LDLIBS += -lglfw -lGL -ldl
endif

SRCS := main.cpp src/dashboard.cpp src/primitives.cpp
SRCS += imgui/imgui.cpp imgui/imgui_demo.cpp imgui/imgui_draw.cpp imgui/imgui_tables.cpp imgui/imgui_widgets.cpp
SRCS += imgui/backends/imgui_impl_glfw.cpp imgui/backends/imgui_impl_opengl3.cpp
SRCS += vendor/implot/implot.cpp vendor/implot/implot_items.cpp

TARGET := dashboard

OBJS = $(SRCS:.cpp=.o)

INCFLAGS = -Iimgui -Iimgui/backends -Ivendor/implot -I.

.PHONY: all clean

all: $(TARGET)
	@echo Build complete for $(TARGET)

# Build rules
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCFLAGS) -c -o $@ $<

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) $(LDLIBS)

clean:
	rm -f $(TARGET) $(OBJS)
