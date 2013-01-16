//==-- Picoblaze.h - Top-level interface for Picoblaze representation --*- C++ -*-==//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in
// the LLVM Picoblaze backend.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TARGET_Picoblaze_H
#define LLVM_TARGET_Picoblaze_H

#include "MCTargetDesc/PicoblazeMCTargetDesc.h"
#include "llvm/Target/TargetMachine.h"

namespace PicoblazeCC {
  // Picoblaze specific condition code.
  enum CondCodes {
    COND_E  = 0,  // aka COND_Z
    COND_NE = 1,  // aka COND_NZ
    COND_HS = 2,  // aka COND_C
    COND_LO = 3,  // aka COND_NC
    COND_GE = 4,
    COND_L  = 5,

    COND_INVALID = -1
  };
}

namespace llvm {
  class PicoblazeTargetMachine;
  class FunctionPass;
  class formatted_raw_ostream;

  FunctionPass *createPicoblazeISelDag(PicoblazeTargetMachine &TM,
                                    CodeGenOpt::Level OptLevel);

  FunctionPass *createPicoblazeBranchSelectionPass();

} // end namespace llvm;
#define PR_FUNCTION() do{printf("%s\n",__FUNCTION__);}while(0)
#endif
