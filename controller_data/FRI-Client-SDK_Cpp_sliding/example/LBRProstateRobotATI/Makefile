BASE_DIR	= ../..
include $(BASE_DIR)/build/GNUMake/paths.mak
include $(BASE_DIR)/build/GNUMake/$(TOOLS_MAK)

SOURCES 		= LBRProstateRobotATIApp.cpp \
			      LBRProstateRobotATIClient.cpp \
				  socketHelper.cpp  
  
TARGET 		= LBRProstateRobotATI

CXXFLAGS 	+= -llapack
LDFLAGS 	+= $(LIB_DIR)/libFRIClient.a -lboost_system -lboost_thread

include $(BASE_DIR)/build/GNUMake/rules_examples.mak
