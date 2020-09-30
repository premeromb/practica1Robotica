project(ejemplo1)
cmake_minimum_required(VERSION 2.6)
MESSAGE( STATUS "COMPILING WITH QT5" )
               find_package(Qt5 REQUIRED COMPONENTS Gui Widgets Sql Core OpenGL Xml XmlPatterns )
               set(QT_LIBRARIES Qt5::Gui Qt5::Core Qt5::Widgets Qt5::Sql Qt5::OpenGL Qt5::Xml Qt5::XmlPatterns)

               MACRO(QT_WRAP_CPP output )
                       FOREACH( input_file ${ARGN} )
                               QT5_WRAP_CPP( ${output} ${input_file} )
                       ENDFOREACH( input_file)
                       SET_PROPERTY(SOURCE ${${output}} PROPERTY SKIP_AUTOGEN ON)
               ENDMACRO(QT_WRAP_CPP)

               MACRO (QT_WRAP_UI output )
                       FOREACH( input_file ${ARGN} )
                               MESSAGE( STATUS "Generating header file from ${input_file}")
                               QT5_WRAP_UI( ${output} ${input_file} )
                       ENDFOREACH( input_file)
                       SET_PROPERTY(SOURCE ${${output}} PROPERTY SKIP_AUTOGEN ON)
               ENDMACRO(QT_WRAP_UI)


include_directories(${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})
ADD_DEFINITIONS( -std=c++17 -Wextra)

# Sources set
SET ( SOURCES  ${SOURCES}  ejemplo1.cpp  main.cpp )
# Headers set
SET ( HEADERS  ${HEADERS} ejemplo1.h )

QT_WRAP_CPP( MOC_SOURCES ${HEADERS} )
QT_WRAP_UI( UI_HEADERS counterDlg.ui )

# Specify construction and link process
ADD_EXECUTABLE( ejemplo1 ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES}  ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( ejemplo1 ${LIBS} ${STATIC_LIBS} ${SPECIFIC_LIBS} ${QT_LIBRARIES} pthread)
INSTALL(TARGETS ejemplo1  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )


