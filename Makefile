

PCH=		all-headers.h.pch
HEADERS=	all-headers.h Mat3.h RotationMatrixIntegrator.h RotationSolver.h Vec3.h all-headers.h
SRCS=		main.cpp

APP=		unfair-dice-simulation

CXXFLAGS+=	-std=c++17

$(APP): $(SRCS) $(PCH)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -include-pch all-headers.h.pch -o $(APP) $(SRCS) `pkg-config --cflags --libs openscenegraph`

$(PCH): $(HEADERS)
	$(CXX) $(CXXFLAGS) all-headers.h -o $(PCH) -I/usr/local/include
