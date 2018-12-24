

PCH=		all-headers.h.pch
HEADERS=	all-headers.h Mat3.h Vec3.h RotationMatrixIntegrator.h RotationSolver.h
SRCS=		main.cpp

APP=		unfair-dice-simulation

CXXFLAGS+=	-std=c++17

$(APP): $(SRCS) $(PCH) Makefile
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -include-pch all-headers.h.pch -o $(APP) $(SRCS) `pkg-config --cflags --libs openscenegraph`

$(PCH): $(HEADERS) Makefile
	$(CXX) $(CXXFLAGS) all-headers.h -o $(PCH) -I/usr/local/include
