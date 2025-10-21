
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was QtHostInfoConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../../" ABSOLUTE)

# Use original install prefix when loaded through a "/usr move"
# cross-prefix symbolic link such as /lib -> /usr/lib.
get_filename_component(_realCurr "${CMAKE_CURRENT_LIST_DIR}" REALPATH)
get_filename_component(_realOrig "/usr/lib/i386-linux-gnu/cmake/Qt6HostInfo" REALPATH)
if(_realCurr STREQUAL _realOrig)
  set(PACKAGE_PREFIX_DIR "/usr")
endif()
unset(_realOrig)
unset(_realCurr)

####################################################################################

set(QT6_HOST_INFO_BINDIR "lib/qt6/bin")
set(QT6_HOST_INFO_INCLUDEDIR "include/i386-linux-gnu/qt6")
set(QT6_HOST_INFO_LIBDIR "lib/i386-linux-gnu")
set(QT6_HOST_INFO_MKSPECSDIR "lib/i386-linux-gnu/qt6/mkspecs")
set(QT6_HOST_INFO_ARCHDATADIR "lib/i386-linux-gnu/qt6")
set(QT6_HOST_INFO_PLUGINSDIR "lib/i386-linux-gnu/qt6/plugins")
set(QT6_HOST_INFO_LIBEXECDIR "lib/qt6/libexec")
set(QT6_HOST_INFO_QMLDIR "lib/i386-linux-gnu/qt6/qml")
set(QT6_HOST_INFO_DATADIR "share/qt6")
set(QT6_HOST_INFO_DOCDIR "share/qt6/doc")
set(QT6_HOST_INFO_TRANSLATIONSDIR "share/qt6/translations")
set(QT6_HOST_INFO_SYSCONFDIR "/etc/xdg")
set(QT6_HOST_INFO_EXAMPLESDIR "lib/i386-linux-gnu/qt6/examples")
set(QT6_HOST_INFO_TESTSDIR "tests")
set(QT6_HOST_INFO_DESCRIPTIONSDIR "share/qt6/modules")
set(QT6_HOST_INFO_QMAKE_MKSPEC "linux-g++")
set(QT6_HOST_INFO_ARCH "i386")
set(QT6_HOST_INFO_SUBARCHS "")
set(QT6_HOST_INFO_BUILDABI "i386-little_endian-ilp32")
set(QT6_HOST_INFO_IS_SHARED_LIBS_BUILD "ON")
