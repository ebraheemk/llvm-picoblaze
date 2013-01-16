//===-- PicoblazeMCTargetDesc.h - Picoblaze Target Descriptions -------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides Picoblaze specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef PicoblazeMCTARGETDESC_H
#define PicoblazeMCTARGETDESC_H

namespace llvm {
class Target;

extern Target ThePicoblazeTarget;

} // End llvm namespace

// Defines symbolic names for Picoblaze registers.
// This defines a mapping from register name to register number.
#define GET_REGINFO_ENUM
#include "PicoblazeGenRegisterInfo.inc"

// Defines symbolic names for the Picoblaze instructions.
#define GET_INSTRINFO_ENUM
#include "PicoblazeGenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "PicoblazeGenSubtargetInfo.inc"

#endif
