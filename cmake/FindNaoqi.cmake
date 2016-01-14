FIND_LIBRARY(ALCOMMON alcommon
	${CMAKE_SOURCE_DIR}/../../naoqi/naoqi-sdk-2.1.3.3-linux64/lib
	NO_DEFAULT_PATH
)

FIND_LIBRARY(ALERROR alerror
	${CMAKE_SOURCE_DIR}/../../naoqi/naoqi-sdk-2.1.3.3-linux64/lib
	NO_DEFAULT_PATH
)

FIND_LIBRARY(QI qi
	${CMAKE_SOURCE_DIR}/../../naoqi/naoqi-sdk-2.1.3.3-linux64/lib
	NO_DEFAULT_PATH
)

FIND_LIBRARY(QITYPE qitype
	${CMAKE_SOURCE_DIR}/../../naoqi/naoqi-sdk-2.1.3.3-linux64/lib
	NO_DEFAULT_PATH
)

FIND_LIBRARY(ALPROXIES alproxies
	${CMAKE_SOURCE_DIR}/../../naoqi/naoqi-sdk-2.1.3.3-linux64/lib
	NO_DEFAULT_PATH
)

FIND_LIBRARY(ALVALUE alvalue
	${CMAKE_SOURCE_DIR}/../../naoqi/naoqi-sdk-2.1.3.3-linux64/lib
	NO_DEFAULT_PATH
)

SET(NAOQI_LIBRARIES ${ALCOMMON} ${ALERROR} ${QITYPE} ${QI} ${ALPROXIES} ${ALVALUE})

SET(NAOQI_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../../naoqi/naoqi-sdk-2.1.3.3-linux64/include)

MESSAGE(STATUS "naoqi include path: " ${NAOQI_INCLUDE_DIR})
MESSAGE(STATUS "naoqi libraries: " ${NAOQI_LIBRARIES})

