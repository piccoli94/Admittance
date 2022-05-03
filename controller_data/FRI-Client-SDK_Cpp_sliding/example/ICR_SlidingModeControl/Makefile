BASE_DIR	= ../..
include $(BASE_DIR)/build/GNUMake/paths.mak
include $(BASE_DIR)/build/GNUMake/$(TOOLS_MAK)

SOURCES 		= SlidingModeApp.cpp \
			  SlidingModeClient.cpp \
			  socketHelper.cpp \
			  region_controller.cpp 


INC_DIR +=		/usr/include 



TARGET 		= ICR_SlidingModeControl

CXXFLAGS 	+=  -llapack
LDFLAGS 		+= $(LIB_DIR)/libFRIClient.a -lboost_system -lboost_thread -pthread

include $(BASE_DIR)/build/GNUMake/rules_examples.mak
