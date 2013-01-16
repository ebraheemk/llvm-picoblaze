##===- lib/Target/Picoblaze/Makefile --------------------------*- Makefile -*-===##
# 
#                     The LLVM Compiler Infrastructure
#
# This file is distributed under the University of Illinois Open Source 
# License. See LICENSE.TXT for details.
# 
##===----------------------------------------------------------------------===##

LEVEL = ../../..
LIBRARYNAME = LLVMPicoblazeCodeGen
TARGET = Picoblaze

# Make sure that tblgen is run, first thing.
BUILT_SOURCES = PicoblazeGenRegisterInfo.inc PicoblazeGenInstrInfo.inc \
		PicoblazeGenAsmWriter.inc \
		PicoblazeGenDAGISel.inc PicoblazeGenCallingConv.inc \
		PicoblazeGenSubtargetInfo.inc

DIRS = InstPrinter TargetInfo MCTargetDesc

include $(LEVEL)/Makefile.common

