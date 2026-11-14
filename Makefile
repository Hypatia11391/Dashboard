CXX := g++

PYTHON_SITE := $(shell python3 -c "import site; print(site.getsitepackages()[0])")
NATIVE := $(PYTHON_SITE)/native
WPI := $(NATIVE)/ntcore

CXXFLAGS := -std=c++20 -O2 \
    -I$(NATIVE)/ntcore/include \
    -I$(NATIVE)/wpiutil/include \
    -I$(NATIVE)/wpinet/include \
    -I$(NATIVE)/wpimath/include \
    -I$(NATIVE)/wpilib/include \
    -I$(NATIVE)/wpihal/include \
    -Ivendor/portaudio/include

LDFLAGS := \
    -L$(NATIVE)/ntcore/lib \
    -L$(NATIVE)/wpinet/lib \
    -L$(NATIVE)/wpiutil/lib \
	-L$(NATIVE)/wpimath/lib \
    -Lvendor/portaudio/lib \
    -Lvendor/portaudio/lib/.libs \
    -Wl,-rpath,$(NATIVE)/ntcore/lib \
    -Wl,-rpath,$(NATIVE)/wpinet/lib \
    -Wl,-rpath,$(NATIVE)/wpiutil/lib \
	-Wl,-rpath,$(NATIVE)/wpimath/lib \
    -Wl,-rpath,vendor/portaudio/lib \
    -Wl,-rpath,vendor/portaudio/lib/.libs

LDLIBS := -lntcore -lwpinet -lwpiutil -lwpimath -lportaudio
UNAME_S := $(shell uname -s)
ifneq ($(findstring Darwin,$(UNAME_S)),)
	CXXFLAGS += -I/usr/local/include
	LDFLAGS += -L/usr/local/lib
	LDLIBS += -lglfw3 -framework Cocoa -framework OpenGL -framework IOKit -framework CoreVideo -framework QuartzCore -framework CoreAudio -framework AudioToolbox -framework AudioUnit -framework CoreServices
else
	LDFLAGS += -Wl,--disable-new-dtags
	LDLIBS += -lglfw -lGL -ldl
endif

SRCS := main.cpp src/dashboard.cpp src/primitives.cpp src/networking-server.cpp src/recording-manager.cpp src/sound-feedback.cpp
SRCS += imgui/imgui.cpp imgui/imgui_demo.cpp imgui/imgui_draw.cpp imgui/imgui_tables.cpp imgui/imgui_widgets.cpp
SRCS += imgui/backends/imgui_impl_glfw.cpp imgui/backends/imgui_impl_opengl3.cpp
SRCS += vendor/implot/implot.cpp vendor/implot/implot_items.cpp

TARGET := dashboard

OBJS = $(SRCS:.cpp=.o)

INCFLAGS = -Iimgui -Iimgui/backends -Ivendor/implot -Ivendor/portaudio/include -Ivendor -I.
DEPS = vendor/stb_image.h

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
