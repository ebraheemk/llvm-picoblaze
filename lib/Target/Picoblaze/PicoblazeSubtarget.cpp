//===-- PicoblazeSubtarget.cpp - Picoblaze Subtarget Information ----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the Picoblaze specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "PicoblazeSubtarget.h"
#include "Picoblaze.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "PicoblazeGenSubtargetInfo.inc"

using namespace llvm;

void PicoblazeSubtarget::anchor() { }

PicoblazeSubtarget::PicoblazeSubtarget(const std::string &TT,
                                 const std::string &CPU,
                                 const std::string &FS) :
  PicoblazeGenSubtargetInfo(TT, CPU, FS) {
 PR_FUNCTION();
  std::string CPUName = "generic";

  // Parse features string.
  ParseSubtargetFeatures(CPUName, FS);
}
