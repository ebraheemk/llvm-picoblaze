//===-- PicoblazeTargetInfo.cpp - Picoblaze Target Implementation ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "Picoblaze.h"
#include "llvm/Module.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

Target llvm::ThePicoblazeTarget;

extern "C" void LLVMInitializePicoblazeTargetInfo() { 
  RegisterTarget<Triple::picoblaze> 
    X(ThePicoblazeTarget, "picoblaze", "Picoblaze [huangjielg@gmail.com]");
}
