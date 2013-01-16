//===-- PicoblazeSelectionDAGInfo.cpp - Picoblaze SelectionDAG Info -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the PicoblazeSelectionDAGInfo class.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "Picoblaze-selectiondag-info"
#include "PicoblazeTargetMachine.h"
using namespace llvm;

PicoblazeSelectionDAGInfo::PicoblazeSelectionDAGInfo(const PicoblazeTargetMachine &TM)
  : TargetSelectionDAGInfo(TM) {
	  PR_FUNCTION();
}

PicoblazeSelectionDAGInfo::~PicoblazeSelectionDAGInfo() {
	PR_FUNCTION();
}
