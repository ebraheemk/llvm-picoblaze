//===-- PicoblazeSelectionDAGInfo.h - Picoblaze SelectionDAG Info -----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the Picoblaze subclass for TargetSelectionDAGInfo.
//
//===----------------------------------------------------------------------===//

#ifndef PicoblazeSELECTIONDAGINFO_H
#define PicoblazeSELECTIONDAGINFO_H

#include "llvm/Target/TargetSelectionDAGInfo.h"

namespace llvm {

class PicoblazeTargetMachine;

class PicoblazeSelectionDAGInfo : public TargetSelectionDAGInfo {
public:
  explicit PicoblazeSelectionDAGInfo(const PicoblazeTargetMachine &TM);
  ~PicoblazeSelectionDAGInfo();
};

}

#endif
