###############################################################################
### Paths (include at the beginning but after BASE_DIR initialization)
###############################################################################
TOOLS_MAK			= tools.mak

BIN_DIR				= $(BASE_DIR)/bin
EXP_DIR				= $(BASE_DIR)/example
DOC_DIR				= $(BASE_DIR)/doc
INC_DIR        	= $(BASE_DIR)/include

# USE THESE INCLUDEs FOR SUN_ROBOT_LIB
INC_DIR        	+= /opt/openrobots/include
INC_DIR        	+= /usr/include
INC_DIR        	+= /usr/include/eigen3
INC_DIR         += /opt/ros/melodic/include/
INC_DIR        	+= /home/icaros-pc/Desktop/Piccoli/danilo_ws/devel/include


LIB_DIR				= $(BASE_DIR)/lib
SRC_DIR				= $(BASE_DIR)/src
OBJ_DIR        	= obj


CLIENTBASE_DIR		= $(SRC_DIR)/base
CLIENTLBR_DIR		= $(SRC_DIR)/client_lbr
CLIENTTRAFO_DIR	= $(SRC_DIR)/client_trafo
CONNECTION_DIR		= $(SRC_DIR)/connection
NANOPB_DIR			= $(SRC_DIR)/nanopb-0.2.8
PROTOBUF_DIR 		= $(SRC_DIR)/protobuf
PROTOBUF_GEN_DIR 	= $(SRC_DIR)/protobuf_gen
