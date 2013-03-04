# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

FIND_PACKAGE( Boost REQUIRED serialization)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Create an executable file from sources:
ADD_LIBRARY(LabyrinthReading SHARED ${files})

# Link external libraries
TARGET_LINK_LIBRARIES(LabyrinthReading ${DisCODe_LIBRARIES} )

# Add aother libraries if necessary, for example:
TARGET_LINK_LIBRARIES(LabyrinthReading ${DisCODe_LIBRARIES} ${OpenCV_LIBS} ${Boost_SERIALIZATION_LIBRARY} )

INSTALL_COMPONENT(LabyrinthReading)
