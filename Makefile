

SRCS=	main.cpp

APP=	unfair-dice-simulation

CXXFLAGS+=	-std=c++17

$(APP): $(SRCS)
	@$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $(APP) $(SRCS) `pkg-config --cflags --libs openscenegraph`
